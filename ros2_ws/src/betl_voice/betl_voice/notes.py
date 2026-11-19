"""BETL's notebook: one markdown file, one bullet per note, newest at the bottom.

    # BETL notes

    - 2026-10-20 14:03  buy batteries
    - 2026-10-20 18:40  hotel room is four twelve
"""

import datetime
import os
import re

HEADER = "# BETL notes\n\n"
LINE = re.compile(r"^- (\d{4}-\d{2}-\d{2} \d{2}:\d{2})\s+(.*)$")


class Notebook:
    def __init__(self, path="~/betl_notes/notes.md"):
        self.path = os.path.expanduser(path)

    def add(self, text):
        text = re.sub(r"\s+", " ", text).strip()
        if not text:
            return None
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        with open(self.path, "a", encoding="utf-8") as f:
            if new:
                f.write(HEADER)
            f.write(f"- {stamp}  {text}\n")
        return stamp

    def all(self):
        """[(timestamp, text)] oldest first."""
        if not os.path.exists(self.path):
            return []
        out = []
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                m = LINE.match(line.rstrip("\n"))
                if m:
                    out.append((m.group(1), m.group(2)))
        return out

    def recent(self, n=3):
        return self.all()[-n:]

    def forget_last(self):
        """Remove the newest note (for 'delete that note'). Returns its text or None."""
        if not os.path.exists(self.path):
            return None
        with open(self.path, encoding="utf-8") as f:
            lines = f.readlines()
        for i in range(len(lines) - 1, -1, -1):
            m = LINE.match(lines[i].rstrip("\n"))
            if m:
                del lines[i]
                with open(self.path, "w", encoding="utf-8") as f:
                    f.writelines(lines)
                return m.group(2)
        return None


def spoken_time(stamp):
    """'2026-10-20 14:03' -> 'today at 2:03 PM' / 'yesterday at ...' / 'October 18'."""
    try:
        t = datetime.datetime.strptime(stamp, "%Y-%m-%d %H:%M")
    except ValueError:
        return stamp
    today = datetime.date.today()
    clock = t.strftime("%I:%M %p").lstrip("0")
    if t.date() == today:
        return f"today at {clock}"
    if t.date() == today - datetime.timedelta(days=1):
        return f"yesterday at {clock}"
    return t.strftime("%B %d").replace(" 0", " ")
