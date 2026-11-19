"""Decide what a spoken command is: translate, stop, note, or general chat.

Input is the recognizer's text for everything said after the wake word, which may still start
with the wake word itself (the recognizer gets a little pre-roll audio). Pure functions, no ROS.
"""

import re
from dataclasses import dataclass

WAKE = r"(?:(?:hey|ok|okay|hi)\s+)?(?:beetle|beetles|beatle|beatles|betel|beadle|bettle|betl|bido|bedal)\b[\s,.!?]*"

# up to two stray words before a command word are tolerated (room noise, the tail of the wake
# word); they are often mis-heard as short Chinese syllables, so those count as words too
LEAD = r"^(?:(?:[a-z']+|[\u4e00-\u9fff])\s*){0,2}?\s*"
TRANSLATE_EN = re.compile(
    LEAD + r"(?:(?:can|could|would)\s+you\s+)?(?:please\s+)?transl\w*"
    r"(?:\s+(?:this|that|for me|please))*"
    r"(?:\s+(?:in|into|to)\s+(?:chinese|mandarin|english))?"
    r"[\s,:]*(?P<phrase>.*)$")
# 翻译 (fanyi) plus same-sounding words the recognizer sometimes writes instead
FANYI = r"(?:翻译|翻一|范逸|反义|番译|翻易|凡易)"
TRANSLATE_ZH = re.compile(LEAD + r"(?:请|麻烦)?(?:你)?(?:帮我)?" + FANYI + r"(?:一下)?(?:成(?:中文|英文|英语))?[\s,:：，]*(?P<phrase>.*)$")
HOW_SAY = re.compile(r"^how\s+(?:do\s+(?:you|i)|would\s+you)\s+say\s+(?P<phrase>.+?)\s+in\s+(?:chinese|mandarin)\??$")

STOP_WORDS = {"stop", "halt", "freeze", "stop stop", "stop now", "emergency stop", "停", "停止", "停下",
              "别动", "不要动"}
NOTE = re.compile(LEAD + r"(?:please\s+)?(?:(?:save|take|make|add|write)\s+(?:a\s+|this\s+)?note(?:\s+(?:to\s+(?:self\s+)?|that|saying))?|note\s+that|remember\s+(?:to\s+|that\s+)?)"
                  r"[\s,:]*(?P<body>.*)$")
NOTE_ZH = re.compile(r"^(?:请|帮我)?(?:记(?:一)?下|记录(?:一下)?)[\s,:：，]*(?P<body>.*)$")


# Voice switching. The whole command must be about the voice, so "translate change your voice"
# is still a translation. Payload: toggle | alt (robot voice) | main (clear voice)
VOICE_ALT = re.compile(LEAD + r"(?:please\s+)?(?:use|switch\s+to|change\s+to|go\s+to)\s+(?:the\s+|your\s+)?(?:robot|retro|klatt|computer)\s+voice(?:\s+please)?$")
VOICE_MAIN = re.compile(LEAD + r"(?:please\s+)?(?:use|switch\s+to|change\s+to|go\s+(?:back\s+)?to)\s+(?:the\s+|your\s+)?(?:clear|normal|human|nice|regular|first|other\s+chinese)\s+voice(?:\s+please)?$")
VOICE_TOGGLE = re.compile(LEAD + r"(?:please\s+)?(?:(?:switch|change|swap|toggle|flip)\s+(?:the\s+|your\s+)?voices?|(?:use|try)\s+(?:the\s+|your\s+)?other\s+voice|(?:switch|change)\s+(?:to\s+)?(?:the\s+)?other\s+voice)(?:\s+please)?$")
VOICE_ZH = re.compile(r"^(?:请|帮我)?(?:换|切换|换一个|换个|改)(?:一下)?(?:声音|语音|嗓音)$")
VOICE_ZH_ALT = re.compile(r"^(?:请)?(?:用|换成|切换到)?机器人(?:的)?(?:声音|语音)$")


RESUME = re.compile(LEAD + r"(?:please\s+)?(?:resume|continue|carry\s+on|all\s+clear|unpause|un\s+pause|start\s+again|restart|you\s+can\s+(?:move|go)(?:\s+again)?|release|go\s+ahead)\b.*$")
RESUME_ZH = re.compile(r"^(?:请)?(?:继续|恢复|可以动了|解除|开始吧)")
NOTES_READ = re.compile(LEAD + r"(?:please\s+)?(?:read|tell|give)\s+(?:me\s+)?(?:back\s+)?(?:all\s+)?(?:my|the|your)?\s*(?:last|latest|recent|newest)?\s*notes?(?:\s+back)?$"
                        r"|^what\s+(?:are|were|is)\s+(?:my|the)\s+(?:last|latest|recent)?\s*notes?$"
                        r"|^what\s+did\s+i\s+(?:just\s+)?(?:note|save)$")
NOTES_READ_ZH = re.compile(r"^(?:请)?(?:读|念)(?:一下)?(?:我的)?(?:最近的)?笔记")
NOTES_DELETE = re.compile(LEAD + r"(?:please\s+)?(?:delete|forget|remove|erase|scratch)\s+(?:that|the\s+last|my\s+last|the\s+latest)\s+note$")
RECALL = re.compile(
    r"^(?:(?:can|could)\s+you\s+)?(?:please\s+)?"
    r"(?:search\s+(?:my\s+|the\s+|your\s+)?(?:notes|files|stuff)?\s*(?:for|about)"
    r"|what\s+(?:did|do)\s+i\s+(?:note|write|save|say|have)\s+(?:down\s+)?(?:about|on|for)"
    r"|what\s+do\s+(?:my|the)\s+(?:notes|files)\s+say\s+(?:about|on)"
    r"|do\s+i\s+have\s+(?:a\s+|any\s+)?(?:notes?|anything)\s+(?:about|on|for)"
    r"|look\s+up|find|check\s+my\s+notes\s+(?:for|about)|remind\s+me\s+(?:about|of|what)?)"
    r"\s+(?P<q>.+)$")
RECALL_ZH = re.compile(r"^(?:请|帮我)?(?:查一下|查查|搜索|搜一下|找一下|查找)(?P<q>.+)$")


@dataclass
class Intent:
    kind: str            # translate | voice | stop | resume | note | notes_read | notes_delete | recall | chat | empty
    text: str = ""       # payload: phrase to translate, note body, or chat text
    raw: str = ""


def normalize(text):
    t = text.strip()
    # English from the recognizer is upper case; Chinese has no case. Lower-case everything.
    t = t.lower()
    t = re.sub(r"\s+", " ", t)
    t = re.sub("^" + WAKE, "", t)
    # the wake word can also be swallowed into the start of the next word, e.g. "beetleplease"
    t = re.sub(r"^(?:beetle|beatle)(?=\w)", "", t)
    return t.strip(" ,.!?")


def parse(text):
    raw = text
    t = normalize(text)
    if not t:
        return Intent("empty", raw=raw)
    if t in STOP_WORDS or t.startswith("stop ") or t.startswith("停"):
        return Intent("stop", raw=raw)
    if RESUME.match(t) or RESUME_ZH.match(t):
        return Intent("resume", raw=raw)
    for rx in (HOW_SAY, TRANSLATE_EN, TRANSLATE_ZH):
        m = rx.match(t)
        if m:
            return Intent("translate", m.group("phrase").strip(" ,.:"), raw)
    if VOICE_ALT.match(t) or VOICE_ZH_ALT.match(t):
        return Intent("voice", "alt", raw)
    if VOICE_MAIN.match(t):
        return Intent("voice", "main", raw)
    if VOICE_TOGGLE.match(t) or VOICE_ZH.match(t):
        return Intent("voice", "toggle", raw)
    if NOTES_READ.match(t) or NOTES_READ_ZH.match(t):
        return Intent("notes_read", raw=raw)
    if NOTES_DELETE.match(t):
        return Intent("notes_delete", raw=raw)
    for rx in (RECALL, RECALL_ZH):
        m = rx.match(t)
        if m:
            return Intent("recall", m.group("q").strip(" ,.?"), raw)
    for rx in (NOTE, NOTE_ZH):
        m = rx.match(t)
        if m:
            return Intent("note", m.group("body").strip(), raw)
    return Intent("chat", t, raw)
