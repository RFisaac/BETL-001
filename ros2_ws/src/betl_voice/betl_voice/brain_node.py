"""brain_node: decides what to do with each spoken command, and owns the voice stop state.

Subscribes:
  wake        std_msgs/String  (listener)  -> show 'listening'
  command     std_msgs/String  (listener)  JSON {"text", "lang"}
  stop        std_msgs/Bool    (listener)  "stop" heard: halt immediately
  speaking    std_msgs/Bool    (tts)       used to time follow-up questions
Publishes:
  estop       std_msgs/Bool    LATCHED (transient local) + 5 Hz heartbeat. true = halt.
                               Motion code should obey it AND treat a missing heartbeat as halt.
  say         std_msgs/String  JSON {"text", "lang"} for tts_node
  state       std_msgs/String  idle | listen | think | stop   (for the LED ring)
  listen      std_msgs/String  asks listener_node for another command without a wake word
  voice_mode  std_msgs/String  main | alt, to tts_node
  transcript  std_msgs/String  one human-readable line per exchange (`ros2 topic echo`)

What BETL understands (after "Beetle"):
  translate   "please translate where is the train station" / "请翻译 …" / "translate" (asks)
  voice       "switch voices" / "use the robot voice" / "use the clear voice" / 换个声音
  stop        "stop" — works WITHOUT the wake word, even mid-sentence. Cuts speech, red ring,
              estop=true, and ignores everything except "Beetle, resume" (or "continue",
              "all clear", 继续).
  note        "save a note buy batteries" / "take a note" (asks) / 记一下 …
              "read my notes", "delete that note"
  recall      "search my notes for …", "what did I note about …", "look up …", 查一下 …
              searches notes.md and every .md/.txt file in the knowledge folder
  chat        anything else: local LLM if llama-server is running (with relevant notes as
              context), otherwise a few canned answers
"""

import json
import os
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from . import router
from .chat import Chat
from .knowledge import Knowledge, speakable
from .notes import Notebook, spoken_time
from .translator import Translator, is_chinese

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain")
        p = self.declare_parameter
        p("models_dir", os.path.expanduser("~/betl_models"))
        p("translate_threads", 2)
        p("notes_file", "~/betl_notes/notes.md")
        p("knowledge_dir", "~/betl_knowledge")
        p("recall_min_score", 2.0)       # how relevant a passage must be to be used
        p("llm_url", "http://127.0.0.1:8080")
        p("llm_timeout_s", 25.0)
        p("llm_max_tokens", 90)
        g = lambda n: self.get_parameter(n).value

        md = os.path.expanduser(g("models_dir"))
        self.translator = Translator(os.path.join(md, "argos"), threads=g("translate_threads"))
        try:
            self.translator.warm_up()
        except Exception as e:
            self.get_logger().error(f"translation models not loaded: {e}")
        self.notes = Notebook(g("notes_file"))
        self.knowledge = Knowledge(g("notes_file"), g("knowledge_dir"))
        self.chat = Chat(g("llm_url"), timeout=g("llm_timeout_s"), max_tokens=g("llm_max_tokens"),
                         log=self.get_logger().info)

        self.pub_say = self.create_publisher(String, "say", 10)
        self.pub_state = self.create_publisher(String, "state", 10)
        self.pub_listen = self.create_publisher(String, "listen", 10)
        self.pub_log = self.create_publisher(String, "transcript", 10)
        self.pub_voice = self.create_publisher(String, "voice_mode", 10)
        self.pub_estop = self.create_publisher(Bool, "estop", LATCHED)
        self.pub_tts_stop = self.create_publisher(Bool, "stop", 10)
        self.create_subscription(String, "wake", self.on_wake, 10)
        self.create_subscription(String, "command", self.on_command, 10)
        self.create_subscription(Bool, "stop", self.on_stop, 10)
        self.create_subscription(Bool, "speaking", self.on_speaking, 10)

        self.stopped = False
        self.zh_mode = "main"
        self.pending = None              # follow-up we are waiting for: 'translate' | 'note'
        self.listen_after_speech = False
        self.lock = threading.Lock()
        self.pub_estop.publish(Bool(data=False))
        self.create_timer(0.2, lambda: self.pub_estop.publish(Bool(data=self.stopped)))
        self.set_state("idle")
        self.get_logger().info(f"brain ready (notes: {self.notes.path}, "
                               f"knowledge: {self.knowledge.folder})")

    # ---- helpers ---------------------------------------------------------------------------
    def set_state(self, s):
        if self.stopped and s in ("idle", "think"):
            s = "stop"
        self.pub_state.publish(String(data=s))

    def say(self, text, lang="en", then_listen=False, even_if_stopped=False):
        if self.stopped and not even_if_stopped:
            return                       # e.g. a translation that finished after "stop"
        self.listen_after_speech = then_listen
        msg = {"text": text, "lang": lang}
        if even_if_stopped:
            msg["keep"] = True           # must not be swallowed by the stop it reports
        self.pub_say.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def say_mixed(self, text):
        self.say(text, "zh" if is_chinese(text) else "en")

    def log(self, line):
        self.get_logger().info(line)
        self.pub_log.publish(String(data=line))

    def in_background(self, fn):
        def run():
            try:
                fn()
            except Exception as e:
                self.get_logger().error(f"{fn.__name__} failed: {e}")
                self.say("Something went wrong.", "en")
            self.set_state("idle")
        self.set_state("think")
        threading.Thread(target=run, daemon=True).start()

    # ---- stop / resume ---------------------------------------------------------------------
    def on_stop(self, msg):
        if msg.data and not self.stopped:
            self.enter_stop("voice")

    def enter_stop(self, why):
        with self.lock:
            self.stopped = True
            self.pending = None
            self.listen_after_speech = False
        self.pub_estop.publish(Bool(data=True))
        if why != "voice":
            self.pub_tts_stop.publish(Bool(data=True))  # the listener already told tts for spoken stops
        self.set_state("stop")
        self.log(f"STOP ({why}): estop on, waiting for 'Beetle, resume'")
        self.say("Halted.", "en", even_if_stopped=True)

    def resume(self):
        with self.lock:
            self.stopped = False
        self.pub_estop.publish(Bool(data=False))
        self.log("RESUME: estop off")
        self.set_state("idle")
        self.say("Resuming.", "en")

    # ---- callbacks -------------------------------------------------------------------------
    def on_wake(self, msg):
        self.pending = None
        self.set_state("listen")         # shown even when stopped, so you know it heard you

    def on_speaking(self, msg):
        if not msg.data and self.listen_after_speech and not self.stopped:
            self.listen_after_speech = False
            self.set_state("listen")
            self.pub_listen.publish(String(data=self.pending or "follow-up"))

    def on_command(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"text": msg.data, "lang": "zh" if is_chinese(msg.data) else "en"}
        text, lang = data.get("text", ""), data.get("lang", "en")
        intent = router.parse(text)

        if self.stopped:
            if intent.kind == "resume":
                return self.resume()
            if intent.kind != "empty":
                self.log(f"stopped: ignoring '{text}' (say 'Beetle, resume')")
            return self.set_state("stop")
        if intent.kind == "stop":
            return self.enter_stop("command")

        # an answer to our own follow-up question goes straight to that feature
        if self.pending:
            what, self.pending = self.pending, None
            phrase = router.normalize(text)
            if not phrase:
                return self.set_state("idle")
            if what == "translate":
                return self.do_translate(phrase)
            if what == "note":
                return self.save_note(phrase, lang)

        k = intent.kind
        if k == "empty":
            self.log("wake word but no command")
            self.set_state("idle")
        elif k == "resume":
            self.say("I am not stopped.", "en")
            self.set_state("idle")
        elif k == "translate":
            if intent.text:
                self.do_translate(intent.text)
            else:
                self.ask("translate", "What should I translate?", "请说要翻译的话。", lang)
        elif k == "voice":
            self.switch_voice(intent.text)
        elif k == "note":
            if intent.text:
                self.save_note(intent.text, lang)
            else:
                self.ask("note", "What should I note?", "要记什么？", lang)
        elif k == "notes_read":
            self.read_notes()
        elif k == "notes_delete":
            gone = self.notes.forget_last()
            self.log(f"note deleted: {gone!r}")
            self.say(f"Deleted: {gone}" if gone else "There are no notes to delete.", "en")
            self.set_state("idle")
        elif k == "recall":
            self.in_background(lambda: self.recall(intent.text))
        else:
            self.in_background(lambda: self.converse(intent.text, lang))

    # ---- features --------------------------------------------------------------------------
    def ask(self, what, en, zh, lang):
        self.pending = what
        self.log(f"{what}: asking")
        self.say(zh if lang == "zh" else en, lang, then_listen=True)
        self.set_state("idle")

    def switch_voice(self, mode):
        """Swap BETL's Chinese voice between the clear one and the robot one, then demo it."""
        if mode == "toggle":
            mode = "alt" if self.zh_mode == "main" else "main"
        self.zh_mode = mode
        self.pub_voice.publish(String(data=mode))
        name = "robot voice" if mode == "alt" else "clear voice"
        self.log(f"voice: Chinese now uses the {name}")
        self.say(f"Switching to the {name}.", "en")
        self.say("你好，我是比特尔。", "zh")
        self.set_state("idle")

    def do_translate(self, phrase):
        def translate():
            out, src, tgt = self.translator.translate(phrase)
            self.log(f"translate [{src}->{tgt}] '{phrase}' -> '{out}'")
            self.say(out, tgt)
        self.in_background(translate)

    def save_note(self, body, lang):
        stamp = self.notes.add(body)
        self.log(f"note saved {stamp}: {body}")
        if lang == "zh":
            self.say("记下了：" + body, "zh")
        else:
            self.say("Noted.", "en")
            self.say_mixed(body)
        self.set_state("idle")

    def read_notes(self):
        recent = self.notes.recent(3)
        total = len(self.notes.all())
        if not recent:
            self.say("You have no notes yet.", "en")
        else:
            self.say(f"You have {total} note{'s' if total != 1 else ''}. "
                     f"{'Here are the latest.' if total > 3 else ''}", "en")
            for stamp, body in recent:
                self.say(f"{spoken_time(stamp).capitalize()}:", "en")
                self.say_mixed(body)
        self.set_state("idle")

    def recall(self, query):
        hits = self.knowledge.search(query, k=2)
        good = [h for h in hits if h[0] >= self.get_parameter("recall_min_score").value]
        self.log(f"recall '{query}': " + ("; ".join(f"{s:.1f} [{src}] {txt[:60]}" for s, _, txt, src in hits)
                                          or "nothing"))
        if not good:
            return self.say(f"I don't have anything about {query}." if not is_chinese(query)
                            else "我没有找到相关的内容。", "zh" if is_chinese(query) else "en")
        score, heading, text, src = good[0]
        if src != "notes":
            self.say(f"From {src.replace('_', ' ')}:", "en")
        self.say_mixed(speakable(heading, text))

    def converse(self, text, lang):
        hits = self.knowledge.search(text, k=3)
        context = [speakable(h, t) for s, h, t, src in hits
                   if s >= self.get_parameter("recall_min_score").value]
        answer, alang = self.chat.reply(text, lang, context)
        self.log(f"chat: '{text}' -> '{answer}'" + (f" (used {len(context)} notes)" if context else ""))
        self.say(answer, alang)


def main():
    rclpy.init()
    node = BrainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
