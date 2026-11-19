"""Text to speech for BETL.

Two engines:
  - espeak-ng (classic formant synthesizer). Named in config as "espeak:<voice>", e.g.
    "espeak:en-us+klatt" (the retro DECtalk-style voice) or "espeak:cmn-latn-pinyin+klatt" for
    Mandarin. Chinese text is converted to pinyin first (pypinyin), because espeak's own
    character dictionary reads the tone numbers out loud in English.
  - Piper neural voices via sherpa-onnx, named by model folder, e.g. "vits-piper-en_US-ryan-low".

Any of them can go through the Speak & Spell LPC effect (style "lpc"), a bit-crusher ("simple"),
or be played as-is ("clean").
"""

import glob
import os
import re
import subprocess
import tempfile
import threading
import wave

import numpy as np

from . import lpc_voice


# 2+ single letters, each its own token, separated by dots, dashes or spaces:
# "R. O. B. O. T.", "R-O-B-O-T", "r o b o t", "U.S.A."
_LETTER_RUN = re.compile(r"(?<![A-Za-z])[A-Za-z](?:(?:\.\s*|-|\s+)[A-Za-z](?![A-Za-z])){1,}\.?")


def clean_english(text):
    """Piper's English voices mumble single letters written as 'B.' or 'T.', so spelled-out
    letters are rewritten as 'R, O, B, O, T.', which they read clearly."""
    def fix(m):
        letters = re.findall(r"[A-Za-z]", m.group(0))
        nxt = m.string[m.end():m.end() + 1]
        end = "" if nxt in "?!,;:" and nxt else "."
        return ", ".join(c.upper() for c in letters) + end
    out = _LETTER_RUN.sub(fix, text)
    return re.sub(r"\s+", " ", out).replace("..", ".").strip()


def _load_piper(model_dir, threads):
    import sherpa_onnx as so
    onnx = sorted(glob.glob(os.path.join(model_dir, "*.onnx")))
    if not onnx:
        raise FileNotFoundError(f"no .onnx voice in {model_dir}")
    cfg = so.OfflineTtsConfig(model=so.OfflineTtsModelConfig(
        vits=so.OfflineTtsVitsModelConfig(
            model=onnx[0], tokens=os.path.join(model_dir, "tokens.txt"),
            data_dir=os.path.join(model_dir, "espeak-ng-data")),
        num_threads=threads))
    return so.OfflineTts(cfg)


class PiperEngine:
    def __init__(self, model_dir, threads):
        self.tts = _load_piper(model_dir, threads)

    def generate(self, text, speed):
        g = self.tts.generate(text, sid=0, speed=speed)
        return np.asarray(g.samples, dtype=np.float32), g.sample_rate


def to_pinyin(text):
    """Chinese characters -> 'ni3 hao3, wo3 shi4 ...' for espeak's pinyin voice."""
    from pypinyin import Style, lazy_pinyin
    out = " ".join(lazy_pinyin(text, style=Style.TONE3, neutral_tone_with_five=True))
    for zh, en in (("，", ","), ("。", "."), ("？", "?"), ("！", "!"), ("、", ","), ("：", ":"), ("；", ";")):
        out = out.replace(zh, en)
    return re.sub(r"\s+([,.?!:;])", r"\1", out)


class EspeakEngine:
    def __init__(self, voice, pitch=30, wpm=145):
        self.voice, self.pitch, self.wpm = voice, int(pitch), int(wpm)
        self.pinyin = "pinyin" in voice
        if subprocess.run(["espeak-ng", "--version"], capture_output=True).returncode != 0:
            raise RuntimeError("espeak-ng is not installed (sudo apt install espeak-ng)")

    def generate(self, text, speed):
        if self.pinyin:
            text = to_pinyin(text)
        with tempfile.NamedTemporaryFile(suffix=".wav") as f:
            subprocess.run(["espeak-ng", "-v", self.voice, "-p", str(self.pitch),
                            "-s", str(int(self.wpm * speed)), "-w", f.name, text],
                           check=True, capture_output=True)
            with wave.open(f.name) as w:
                rate = w.getframerate()
                a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        return a.astype(np.float32) / 32768, rate


def make_engine(name, models_dir, threads, pitch, wpm):
    if name.startswith("espeak:"):
        return EspeakEngine(name.split(":", 1)[1], pitch, wpm)
    return PiperEngine(os.path.join(models_dir, name), threads)


class Voice:
    def __init__(self, en_voice, zh_voice, models_dir="", threads=2, en_style="clean",
                 zh_style="clean", speed=1.0, pitch=1.0, monotone=0.25, breath=0.0,
                 espeak_pitch=30, espeak_wpm=145, espeak_zh_wpm=130, zh_voice_alt=""):
        self.en = make_engine(en_voice, models_dir, threads, espeak_pitch, espeak_wpm)
        try:
            self.zh = make_engine(zh_voice, models_dir, threads, espeak_pitch, espeak_zh_wpm) \
                if zh_voice else None
        except Exception:
            self.zh = None
        # a second Chinese voice that the "switch voices" command toggles to
        try:
            self.zh_alt = make_engine(zh_voice_alt, models_dir, threads, espeak_pitch,
                                      espeak_zh_wpm) if zh_voice_alt else None
        except Exception:
            self.zh_alt = None
        self.zh_mode = "main"           # main | alt
        self.en_style, self.zh_style = en_style, zh_style
        self.speed, self.pitch, self.monotone, self.breath = speed, pitch, monotone, breath
        self.lock = threading.Lock()

    def set_zh_mode(self, mode):
        """mode: main | alt | toggle. Returns the mode now in use."""
        if mode == "toggle":
            mode = "alt" if self.zh_mode == "main" else "main"
        if mode == "alt" and not self.zh_alt:
            mode = "main"
        self.zh_mode = mode
        return mode

    def synth(self, text, lang="en"):
        """Returns (float32 mono audio, sample_rate)."""
        lang = "zh" if lang.startswith("zh") else "en"
        zh = self.zh_alt if (self.zh_mode == "alt" and self.zh_alt) else self.zh
        eng = zh if (lang == "zh" and zh) else self.en
        style = self.zh_style if lang == "zh" else self.en_style
        if lang == "en":
            text = clean_english(text)
        with self.lock:
            audio, rate = eng.generate(text, self.speed if lang == "en" else 1.0)
        if len(audio) == 0:
            return audio, rate
        if style == "lpc":
            audio = lpc_voice.speak_and_spell(audio, rate, pitch_scale=self.pitch,
                                              monotone=self.monotone, breath=self.breath)
        elif style == "simple":
            audio = lpc_voice.simple_robot(audio, rate)
        else:
            audio = audio / (np.max(np.abs(audio)) + 1e-9) * 0.9
        return audio.astype(np.float32), rate
