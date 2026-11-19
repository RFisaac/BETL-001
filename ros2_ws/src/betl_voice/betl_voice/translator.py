"""Offline English <-> Chinese translation.

Uses the Argos Translate 1.9 model packages (CTranslate2 + SentencePiece) directly, without the
argostranslate library and its heavy dependencies. scripts/download_models.sh unpacks them to
<models_dir>/argos/translate-en_zh-1_9 and translate-zh_en-1_9.
"""

import os
import re

CJK = re.compile(r"[㐀-䶿一-鿿豈-﫿]")
QUESTION_WORDS = ("where", "what", "when", "who", "why", "how", "which", "is", "are", "can",
                  "could", "do", "does", "did", "will", "would", "should", "may")


def is_chinese(text):
    return bool(CJK.search(text))


def tidy_english(text):
    """Speech recognizer output is ALL CAPS with no punctuation; the translator does better
    with ordinary sentence case."""
    t = text.strip().lower()
    if not t:
        return t
    t = re.sub(r"\bi\b", "I", t)
    t = t[0].upper() + t[1:]
    if t[-1] not in ".?!":
        t += "?" if t.split()[0].lower() in QUESTION_WORDS else "."
    return t


def _detok(pieces, chinese):
    s = "".join(pieces).replace("▁", " ").strip()
    s = re.sub(r"\s+", " ", s)
    if chinese:
        s = re.sub(r"(?<=[　-鿿＀-￯])\s+|\s+(?=[　-鿿＀-￯])", "", s)
        s = s.replace("?", "？").replace("!", "！").replace(",", "，")
    else:
        s = re.sub(r"\s+([?.!,])", r"\1", s)
    return s


class Translator:
    def __init__(self, argos_dir, threads=2, beam_size=2):
        self.argos_dir = argos_dir
        self.threads = threads
        self.beam_size = beam_size
        self._models = {}

    def _load(self, pair):
        if pair not in self._models:
            import ctranslate2
            import sentencepiece as spm
            d = os.path.join(self.argos_dir, f"translate-{pair}-1_9")
            if not os.path.isdir(d):
                cands = [x for x in os.listdir(self.argos_dir) if x.startswith(f"translate-{pair}")]
                if not cands:
                    raise FileNotFoundError(f"no translation model for {pair} in {self.argos_dir}")
                d = os.path.join(self.argos_dir, cands[0])
            sp = spm.SentencePieceProcessor(model_file=os.path.join(d, "sentencepiece.model"))
            tr = ctranslate2.Translator(os.path.join(d, "model"), device="cpu",
                                        intra_threads=self.threads, compute_type="int8")
            self._models[pair] = (sp, tr)
        return self._models[pair]

    def translate(self, text, src=None, tgt=None):
        """Returns (translation, src_lang, tgt_lang). Direction is picked from the text when
        src/tgt are not given: anything with Chinese characters goes to English."""
        if src is None:
            src = "zh" if is_chinese(text) else "en"
        if tgt is None:
            tgt = "en" if src == "zh" else "zh"
        if src == "en":
            text = tidy_english(text)
        else:
            text = text.replace(" ", "")
        sp, tr = self._load(f"{src}_{tgt}")
        out = tr.translate_batch([sp.encode(text, out_type=str)], beam_size=self.beam_size,
                                 max_decoding_length=200)
        return _detok(out[0].hypotheses[0], chinese=(tgt == "zh")), src, tgt

    def warm_up(self):
        for pair in ("en_zh", "zh_en"):
            try:
                self._load(pair)
            except FileNotFoundError:
                pass
