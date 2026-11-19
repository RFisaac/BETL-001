"""BETL's conversation.

If a local LLM server is running (llama.cpp's `llama-server`, which speaks the OpenAI chat API on
http://127.0.0.1:8080), questions go to it along with any notes or files that look relevant.
If it isn't running, a few canned answers keep things working. Everything stays on the robot.

reply(text, lang, context) -> (answer_text, answer_lang)
"""

import datetime
import json
import re
import urllib.request
from collections import deque

PERSONA = (
    "You are BETL (pronounced 'Beetle'), a small friendly robot built by Isaac. You talk through a "
    "retro robot voice, so keep every answer short: one or two plain sentences, no lists, no "
    "markdown, no emoji. Spell out symbols as words. If you are given notes or files, use them when "
    "they answer the question and say so briefly; if they don't, say you don't know rather than "
    "guessing. Answer in the same language as the question (English or Chinese)."
)


class Chat:
    def __init__(self, url="http://127.0.0.1:8080", timeout=25.0, max_tokens=90, history=3,
                 log=print):
        self.url = url.rstrip("/") + "/v1/chat/completions"
        self.timeout = timeout
        self.max_tokens = max_tokens
        self.history = deque(maxlen=history * 2)
        self.log = log
        self.llm_ok = None

    def _ask_llm(self, text, context):
        sys = PERSONA
        if context:
            sys += "\n\nNotes and files that might help:\n" + "\n".join(f"- {c}" for c in context)
        now = datetime.datetime.now()
        sys += (f"\n\n(Only if the user asks about the current date or time: it is "
                f"{now:%A %B %d %Y}, {now:%I:%M %p}.)")
        msgs = [{"role": "system", "content": sys}] + list(self.history) + [{"role": "user", "content": text}]
        body = json.dumps({"messages": msgs, "max_tokens": self.max_tokens, "temperature": 0.6,
                           "stream": False}).encode()
        req = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            out = json.loads(r.read())
        answer = out["choices"][0]["message"]["content"].strip()
        answer = re.sub(r"[*_#`]+", "", answer)
        answer = re.sub(r"\s+", " ", answer)
        answer = first_sentences(answer, 2)
        self.history.append({"role": "user", "content": text})
        self.history.append({"role": "assistant", "content": answer})
        return answer

    def reply(self, text, lang="en", context=()):
        try:
            answer = self._ask_llm(text, list(context))
            if self.llm_ok is not True:
                self.log("chat: local LLM is answering")
            self.llm_ok = True
            if answer:
                return answer, ("zh" if re.search(r"[一-鿿]", answer) else "en")
        except Exception as e:
            if self.llm_ok is not False:
                self.log(f"chat: no LLM at {self.url} ({e.__class__.__name__}); using canned answers")
            self.llm_ok = False
        return canned(text, lang, context)


def first_sentences(text, n=2):
    """Small models ramble; a robot voice should not. Keep the first n sentences."""
    parts = re.split(r"(?<=[.!?。！？])\s*", text.strip())
    parts = [p for p in parts if p]
    if not parts:
        return text
    joiner = "" if re.search(r"[\u4e00-\u9fff]", text) else " "
    return joiner.join(parts[:n])


def canned(text, lang="en", context=()):
    t = text.lower().strip()
    if lang == "zh":
        if re.search(r"你好|嗨|哈喽", t):
            return "你好！我是比特尔。", "zh"
        if re.search(r"几点|时间", t):
            return f"现在是{datetime.datetime.now():%H点%M分}。", "zh"
        return "我还不会聊天，但我可以帮你翻译。", "zh"
    if re.search(r"\b(hello|hi|hey|good (morning|afternoon|evening))\b", t):
        return "Hello. I am Beetle.", "en"
    if re.search(r"\bwhat time\b|\bthe time\b", t):
        return f"It is {datetime.datetime.now():%I:%M %p}.".replace(" 0", " "), "en"
    if re.search(r"\bwhat day\b|\bdate\b", t):
        return f"Today is {datetime.datetime.now():%A, %B %d}.".replace(" 0", " "), "en"
    if re.search(r"\byour name\b|\bwho are you\b", t):
        return "I am Beetle, a robot.", "en"
    if context:
        return f"From my notes: {context[0]}", "en"
    return "I cannot chat yet, but I can translate, take notes, and look things up.", "en"
