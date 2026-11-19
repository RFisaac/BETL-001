"""Find things in BETL's notes and in any text files you give it.

Sources: the notes file plus every .md / .txt file under the knowledge folder (sub-folders too):
a travel agenda, wiki pages exported to markdown, packing lists, phrase lists...

Each file is cut into small passages (a bullet, a table row, or a paragraph, each keeping the
heading it sits under) and searched with BM25, the classic keyword ranking used by search
engines. Chinese is handled as single characters plus character pairs. The index rebuilds
itself whenever a file changes, so you can drop new files in while BETL runs.
"""

import glob
import math
import os
import re
from collections import Counter

STOP = set("""a an the and or but of to in on at for with by from is are was were be been am i you
he she it we they me my your our their this that these those what which who whom when where why how
do does did can could would should will shall may might must have has had there here about into
please beetle hey ok okay tell show find search look up note notes say said said mine any some
""".split())
CJK = re.compile(r"[一-鿿]")
NUM_WORDS = {"zero": "0", "one": "1", "two": "2", "three": "3", "four": "4", "five": "5", "six": "6",
             "seven": "7", "eight": "8", "nine": "9", "ten": "10"}


def tokens(text):
    text = text.lower()
    out = []
    for w in re.findall(r"[a-z0-9']+|[一-鿿]+", text):
        if CJK.match(w):
            out.extend(w)                                  # single characters
            out.extend(w[i:i + 2] for i in range(len(w) - 1))   # and pairs
        else:
            w = w.strip("'")
            w = NUM_WORDS.get(w, w)
            if w and w not in STOP:
                out.append(w)
                if len(w) > 4 and w.endswith("s"):
                    out.append(w[:-1])                     # crude plural folding
    return out


def strip_markdown(text):
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)     # [label](url) -> label
    text = re.sub(r"[*_`]+", "", text)
    text = re.sub(r"[#>|]+", " ", text)
    text = re.sub(r"^\s*[-+]\s+", "", text, flags=re.M)
    text = re.sub(r"\s+([,.:;!?])", r"\1", text)
    return re.sub(r"\s+", " ", text).strip()


def split_passages(text, source):
    """Split a markdown/text file into passages, each tagged with its nearest heading."""
    passages = []
    heading = ""
    para = []

    def flush():
        if para:
            body = " ".join(para).strip()
            if body:
                passages.append((heading, body, source))
            para.clear()

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            flush()
            continue
        if line.startswith("#"):
            flush()
            heading = line.lstrip("#").strip()
            continue
        if re.match(r"^([-*+]|\d+[.)])\s+", line) or line.startswith("|"):
            flush()
            if not re.match(r"^\|[\s:|-]+\|?$", line):           # skip table separator rows
                passages.append((heading, line, source))
            continue
        para.append(line)
    flush()
    return passages


class Knowledge:
    def __init__(self, notes_path, folder):
        self.notes_path = os.path.expanduser(notes_path)
        self.folder = os.path.expanduser(folder)
        self._stamp = None
        self.passages = []

    def files(self):
        fs = []
        if os.path.exists(self.notes_path):
            fs.append(self.notes_path)
        if os.path.isdir(self.folder):
            for ext in ("md", "txt", "markdown"):
                fs.extend(glob.glob(os.path.join(self.folder, "**", f"*.{ext}"), recursive=True))
        return sorted(set(fs))

    def _refresh(self):
        fs = self.files()
        stamp = tuple((f, os.path.getmtime(f)) for f in fs)
        if stamp == self._stamp:
            return
        self._stamp = stamp
        self.passages = []
        for f in fs:
            try:
                with open(f, encoding="utf-8", errors="ignore") as fh:
                    name = "notes" if f == self.notes_path else os.path.splitext(os.path.basename(f))[0]
                    self.passages.extend(split_passages(fh.read(), name))
            except OSError:
                pass
        self.docs = [Counter(tokens(h + " " + b)) for h, b, _ in self.passages]
        self.lens = [sum(d.values()) for d in self.docs]
        self.avg = (sum(self.lens) / len(self.lens)) if self.lens else 1.0
        df = Counter()
        for d in self.docs:
            df.update(d.keys())
        n = len(self.docs)
        self.idf = {t: math.log(1 + (n - c + 0.5) / (c + 0.5)) for t, c in df.items()}

    def search(self, query, k=3, k1=1.2, b=0.75):
        """[(score, heading, text, source)] best first."""
        self._refresh()
        q = tokens(query)
        if not q or not self.passages:
            return []
        scored = []
        for i, d in enumerate(self.docs):
            s = 0.0
            for t in q:
                f = d.get(t, 0)
                if f:
                    s += self.idf.get(t, 0) * f * (k1 + 1) / (f + k1 * (1 - b + b * self.lens[i] / self.avg))
            if s > 0:
                h, txt, src = self.passages[i]
                scored.append((s, h, txt, src))
        scored.sort(key=lambda x: -x[0])
        return scored[:k]


def speakable(heading, text, limit=260):
    """A passage as one short spoken sentence or two."""
    t = strip_markdown(text)
    t = re.sub(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}\s+", "", t)    # note timestamps
    if heading and heading.lower() not in t.lower() and heading.lower() != "betl notes":
        t = f"{strip_markdown(heading)}: {t}"
    if len(t) > limit:
        cut = t[:limit]
        t = cut[:max(cut.rfind(". "), cut.rfind("。"), cut.rfind(", "), limit // 2)] + "."
    return t
