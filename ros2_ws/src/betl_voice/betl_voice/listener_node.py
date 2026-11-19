"""listener_node: microphone -> wake word -> command text.

Publishes:
  wake       std_msgs/String   wake phrase that was heard (e.g. 'BEETLE')
  partial    std_msgs/String   words recognized so far in the current command (for display)
  command    std_msgs/String   JSON {"text": ..., "lang": "en"|"zh"} once the speaker pauses;
                               text is "" if nobody said anything after the wake word
  level      std_msgs/Float32  input loudness in dBFS, ~20 Hz (drives the LED noise blips)
  stop       std_msgs/Bool     true the instant "stop" is heard (no wake word needed, works
                               while BETL is talking); tts_node cuts speech, brain_node halts
Subscribes:
  speaking   std_msgs/Bool     from tts_node; wake words are ignored while BETL talks
  listen     std_msgs/String   from brain_node; opens a command window without a wake word
"""

import json
import math
import os
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from . import audio_io
from .speech import RATE, SpeechEngine
from .translator import is_chinese


class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener")
        p = self.declare_parameter
        p("models_dir", os.path.expanduser("~/betl_models"))
        p("kws_model", "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01")
        p("asr_model", "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16")
        p("asr_precision", "fp32")
        p("wake_phrases", ["BEETLE", "BEEDLE", "HEY BEETLE"])
        p("stop_phrases", ["STOP", "STOP STOP", "BEETLE STOP"])
        p("stop_threshold", 0.06)      # more sensitive than the wake word on purpose
        p("stop_score", 3.0)
        p("kws_threshold", 0.12)
        p("kws_score", 2.0)
        p("threads", 2)
        p("endpoint_silence_s", 1.0)
        p("command_timeout_s", 8.0)
        p("no_speech_timeout_s", 4.0)
        p("input_device", "")          # name substring; empty = find the reSpeaker
        p("input_channel", 0)          # channel 0 = XVF3800 processed (beamformed, cleaned) audio
        p("ignore_wake_while_speaking", True)
        p("input_file", "")            # replay a WAV instead of the mic (testing / tuning)
        g = lambda n: self.get_parameter(n).value

        md = os.path.expanduser(g("models_dir"))
        self.get_logger().info("loading wake word and speech models…")
        t0 = time.time()
        self.engine = SpeechEngine(
            os.path.join(md, g("kws_model")), os.path.join(md, g("asr_model")),
            wake_phrases=list(g("wake_phrases")), stop_phrases=list(g("stop_phrases")),
            stop_threshold=g("stop_threshold"), stop_score=g("stop_score"), threads=g("threads"),
            kws_threshold=g("kws_threshold"), kws_score=g("kws_score"),
            asr_precision=g("asr_precision"), endpoint_silence=g("endpoint_silence_s"),
            command_timeout=g("command_timeout_s"), no_speech_timeout=g("no_speech_timeout_s"))
        self.get_logger().info(f"models loaded in {time.time() - t0:.1f} s")

        self.pub_wake = self.create_publisher(String, "wake", 10)
        self.pub_partial = self.create_publisher(String, "partial", 10)
        self.pub_cmd = self.create_publisher(String, "command", 10)
        self.pub_level = self.create_publisher(Float32, "level", 10)
        self.pub_stop = self.create_publisher(Bool, "stop", 10)
        self.create_subscription(Bool, "speaking", self.on_speaking, 10)
        self.create_subscription(String, "listen", self.on_listen, 10)

        self.speaking = False
        self.speaking_until = 0.0
        self.listen_request = threading.Event()
        self.q = queue.Queue(maxsize=400)
        self.running = True
        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.worker.start()
        self.audio_thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.audio_thread.start()

    # ---- ROS callbacks ---------------------------------------------------------------------
    def on_speaking(self, msg):
        self.speaking = msg.data
        if not msg.data:
            self.speaking_until = time.time() + 0.4      # let the room echo die down

    def on_listen(self, msg):
        self.get_logger().info(f"follow-up listen requested ({msg.data})")
        self.listen_request.set()

    # ---- audio capture ---------------------------------------------------------------------
    def audio_loop(self):
        hint = self.get_parameter("input_device").value
        ch = self.get_parameter("input_channel").value
        if self.get_parameter("input_file").value:
            return self.replay_file(os.path.expanduser(self.get_parameter("input_file").value), ch)
        while self.running:
            dev = audio_io.find_device("input", hint)
            if dev is None:
                self.get_logger().warn("no reSpeaker input found; retrying in 3 s\n" +
                                       audio_io.describe_devices())
                time.sleep(3)
                continue

            def cb(indata, frames, t, status):
                try:
                    self.q.put_nowait(indata[:, min(ch, indata.shape[1] - 1)].copy())
                except queue.Full:
                    pass

            try:
                stream, rate, chans = audio_io.open_stream("input", dev, cb)
            except Exception as e:
                self.get_logger().error(str(e))
                time.sleep(3)
                continue
            self.in_rate = rate
            self.get_logger().info(f"listening on device {dev}: {chans} ch @ {rate} Hz")
            with stream:
                while self.running and stream.active:
                    time.sleep(0.2)
            self.get_logger().warn("audio stream stopped; reopening")

    def replay_file(self, path, ch):
        """Feed a WAV file in real time as if it were the mic, then near-silence."""
        import wave
        with wave.open(path) as w:
            rate, n_ch = w.getframerate(), w.getnchannels()
            raw = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        x = raw.reshape(-1, n_ch)[:, min(ch, n_ch - 1)].astype(np.float32) / 32768
        self.in_rate = rate
        block = rate // 20
        self.get_logger().info(f"replaying {path} ({len(x) / rate:.1f} s) instead of the mic")
        time.sleep(1.0)
        i = 0
        while self.running:
            chunk = x[i:i + block] if i < len(x) else np.random.normal(0, 1e-4, block).astype(np.float32)
            if i < len(x) and len(chunk) < block:
                chunk = np.concatenate([chunk, np.zeros(block - len(chunk), np.float32)])
            i += block
            try:
                self.q.put_nowait(chunk)
            except queue.Full:
                pass
            time.sleep(block / rate)

    # ---- recognition -----------------------------------------------------------------------
    def process_loop(self):
        self.last_level_pub = 0.0
        ignore = self.get_parameter("ignore_wake_while_speaking").value
        while self.running and rclpy.ok():
            try:
                x = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.process_chunk(x, ignore)
            except Exception as e:          # shutting down, or a bad chunk: never kill the loop
                if not rclpy.ok():
                    return
                self.get_logger().error(f"processing error: {e}")

    def process_chunk(self, x, ignore):
        rate = getattr(self, "in_rate", RATE)
        if rate != RATE:
            x = audio_io.resample(x, rate, RATE)
        now = time.time()
        if now - self.last_level_pub > 0.05:
            rms = float(np.sqrt(np.mean(x * x)) + 1e-9)
            self.pub_level.publish(Float32(data=20 * math.log10(rms)))
            self.last_level_pub = now

        if self.listen_request.is_set() and not self.engine.capturing:
            self.listen_request.clear()
            self.engine.start_command(preroll="followup")

        busy_talking = self.speaking or now < self.speaking_until
        self.engine.wake_enabled = not (ignore and busy_talking)
        for kind, value in self.engine.feed(x):
            if kind == "stop":
                self.get_logger().warn(f"STOP heard ({value})")
                self.pub_stop.publish(Bool(data=True))
            elif kind == "wake":
                self.get_logger().info(f"wake word: {value}")
                self.pub_wake.publish(String(data=value))
            elif kind == "partial":
                self.pub_partial.publish(String(data=value))
            elif kind == "command":
                lang = "zh" if is_chinese(value) else "en"
                self.get_logger().info(f"command: '{value}'")
                self.pub_cmd.publish(String(data=json.dumps({"text": value, "lang": lang},
                                                            ensure_ascii=False)))


def main():
    rclpy.init()
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
