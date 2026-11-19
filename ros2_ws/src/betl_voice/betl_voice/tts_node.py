"""tts_node: speaks through the reSpeaker's own output.

Playing through the reSpeaker (not some other speaker) matters: the XVF3800 uses what it plays
as the echo-cancellation reference, so the mics can still hear you while BETL talks. That's
what will make a spoken 'stop' work mid-sentence later.

Subscribes:
  say       std_msgs/String  JSON {"text", "lang", "keep"} (plain text also accepted, as English);
                             keep=true messages are never dropped by a stop
  stop      std_msgs/Bool    true = cut speech now and drop anything queued
  voice_mode std_msgs/String main | alt | toggle: which Chinese voice to use
Publishes:
  speaking  std_msgs/Bool    true while audio is playing
  speech_level std_msgs/Float32  0..1 loudness of what is playing, ~25 Hz (LED speak mode)
"""

import json
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
from .tts_engine import Voice


class TtsNode(Node):
    def __init__(self):
        super().__init__("tts")
        p = self.declare_parameter
        p("models_dir", os.path.expanduser("~/betl_models"))
        p("en_voice", "espeak:en-us+klatt")   # or a Piper folder, e.g. vits-piper-en_US-ryan-low
        p("zh_voice", "vits-piper-zh_CN-huayan-medium")   # clear Chinese voice
        p("zh_voice_alt", "espeak:cmn-latn-pinyin+klatt")  # robot Chinese voice ("switch voices")
        p("en_style", "clean")          # lpc | simple | clean
        p("zh_style", "clean")
        p("espeak_pitch", 30)           # 0-99, espeak default 50
        p("espeak_wpm", 145)            # words per minute for English
        p("espeak_zh_wpm", 130)
        p("speed", 1.0)
        p("pitch", 1.0)
        p("monotone", 0.25)
        p("breath", 0.0)
        p("volume", 0.8)
        p("output_device", "")          # name substring; empty = the reSpeaker
        p("threads", 2)
        p("save_dir", "")               # also save every utterance as a WAV here (debugging)
        g = lambda n: self.get_parameter(n).value
        md = os.path.expanduser(g("models_dir"))
        self.voice = Voice(g("en_voice"), g("zh_voice"), models_dir=md,
                           threads=g("threads"), en_style=g("en_style"), zh_style=g("zh_style"),
                           speed=g("speed"), pitch=g("pitch"), monotone=g("monotone"),
                           breath=g("breath"), espeak_pitch=g("espeak_pitch"),
                           espeak_wpm=g("espeak_wpm"), espeak_zh_wpm=g("espeak_zh_wpm"),
                           zh_voice_alt=g("zh_voice_alt"))
        self.pub_speaking = self.create_publisher(Bool, "speaking", 10)
        self.pub_level = self.create_publisher(Float32, "speech_level", 10)
        self.create_subscription(String, "say", self.on_say, 10)
        self.create_subscription(Bool, "stop", self.on_stop, 10)
        self.create_subscription(String, "voice_mode", self.on_voice_mode, 10)
        self.q = queue.Queue()
        self.abort = threading.Event()
        self.stop_time = 0.0
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info("tts ready")

    def on_say(self, msg):
        try:
            d = json.loads(msg.data)
            text, lang, keep = d.get("text", ""), d.get("lang", "en"), bool(d.get("keep"))
        except json.JSONDecodeError:
            text, lang, keep = msg.data, "en", False
        if text.strip():
            # keep=True (e.g. "Halted.") survives a stop that arrives just after it
            self.q.put((float("inf") if keep else time.time(), text, lang))

    def on_voice_mode(self, msg):
        mode = self.voice.set_zh_mode(msg.data.strip().lower())
        self.get_logger().info(f"Chinese voice is now '{mode}'")

    def on_stop(self, msg):
        """Cut the current sentence and drop everything queued before now. Repeats within a
        second are ignored, so a second copy of the same stop can't swallow 'Halted.'"""
        if not msg.data:
            return
        now = time.time()
        if now - self.stop_time < 1.0:
            return
        self.stop_time = now
        self.abort.set()

    def loop(self):
        dev_hint = self.get_parameter("output_device").value
        while self.running:
            try:
                queued_at, text, lang = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            if queued_at < self.stop_time:
                continue                    # said before a stop: drop it
            self.abort.clear()
            t0 = time.time()
            audio, rate = self.voice.synth(text, lang)
            tag = f"{lang}/{self.voice.zh_mode}" if lang.startswith("zh") else lang
            self.get_logger().info(f"say [{tag}] '{text}' ({len(audio) / rate:.1f} s audio, "
                                   f"made in {time.time() - t0:.2f} s)")
            if self.abort.is_set() or len(audio) == 0:
                continue
            self.save(audio, rate, text)
            self.play(audio, rate, dev_hint)

    def save(self, audio, rate, text):
        d = os.path.expanduser(self.get_parameter("save_dir").value)
        if not d:
            return
        import wave
        os.makedirs(d, exist_ok=True)
        fn = os.path.join(d, time.strftime("%Y%m%d_%H%M%S_") + str(abs(hash(text)) % 10000) + ".wav")
        with wave.open(fn, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes((np.clip(audio, -1, 1) * 32767).astype(np.int16).tobytes())
        self.get_logger().info(f"saved {fn}")

    def play(self, audio, rate, dev_hint):
        try:
            dev = audio_io.find_device("output", dev_hint)
        except Exception as e:
            dev = None
            self.get_logger().error(f"audio library problem: {e}")
        if dev is None:
            # no speaker: still act out the timing so the rest of the system behaves the same
            self.get_logger().warn("no reSpeaker output found; simulating playback")
            self.pub_speaking.publish(Bool(data=True))
            t_end = time.time() + len(audio) / rate
            while time.time() < t_end and not self.abort.is_set():
                time.sleep(0.04)
            self.pub_speaking.publish(Bool(data=False))
            return
        try:
            stream, out_rate, ch = audio_io.open_stream("output", dev, prefer_rate=rate)
        except Exception as e:
            self.get_logger().error(str(e))
            return
        y = audio_io.resample(audio, rate, out_rate) * float(self.get_parameter("volume").value)
        block = out_rate // 25
        self.pub_speaking.publish(Bool(data=True))
        try:
            with stream:
                for i in range(0, len(y), block):
                    if self.abort.is_set():
                        break
                    chunk = y[i:i + block]
                    lvl = float(np.sqrt(np.mean(chunk * chunk)) * 4)
                    self.pub_level.publish(Float32(data=min(1.0, lvl)))
                    stream.write(np.repeat(chunk[:, None], ch, axis=1).astype(np.float32))
        finally:
            self.pub_level.publish(Float32(data=0.0))
            self.pub_speaking.publish(Bool(data=False))


def main():
    rclpy.init()
    node = TtsNode()
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
