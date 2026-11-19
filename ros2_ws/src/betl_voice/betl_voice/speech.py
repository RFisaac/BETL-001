"""Wake word + command transcription, built on sherpa-onnx. No ROS in here so it can be tested
on its own.

Flow: audio is fed in 16 kHz chunks. While idle only the keyword spotter runs. When it hears a
wake phrase, the recent audio (pre-roll) plus everything after it goes to the bilingual
streaming recognizer until the speaker pauses (endpoint) or a timeout, and the text is
returned as a 'command' event.
"""

import os
import tempfile
from collections import deque

import numpy as np

RATE = 16000


def _find(model_dir, *must):
    """First .onnx file in model_dir whose name contains every string in `must`."""
    for f in sorted(os.listdir(model_dir)):
        if f.endswith(".onnx") and all(m in f for m in must):
            return os.path.join(model_dir, f)
    raise FileNotFoundError(f"no {'/'.join(must)} model in {model_dir}")


def build_keywords_file(model_dir, phrases, score=None, threshold=None, extra_groups=()):
    """Turn plain phrases ('HEY BEETLE') into the token format the keyword spotter needs,
    using the model's own BPE model. extra_groups is a list of (phrases, score, threshold) for
    keywords that need their own sensitivity (e.g. STOP). Returns the path of a temp file."""
    import sentencepiece as spm
    sp = spm.SentencePieceProcessor(model_file=os.path.join(model_dir, "bpe.model"))
    lines = []
    for group, sc, th in [(phrases, score, threshold)] + list(extra_groups):
        for p in group:
            p = p.strip().upper()
            if not p:
                continue
            toks = " ".join(sp.encode(p, out_type=str))
            extra = ""
            if sc is not None:
                extra += f" :{sc}"
            if th is not None:
                extra += f" #{th}"
            lines.append(f"{toks}{extra} @{p.replace(' ', '_')}")
    fd, path = tempfile.mkstemp(prefix="betl_kw_", suffix=".txt")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


class SpeechEngine:
    def __init__(self, kws_dir, asr_dir, wake_phrases=("BEETLE", "BEEDLE", "HEY BEETLE"),
                 stop_phrases=("STOP", "STOP STOP", "BEETLE STOP"), stop_threshold=0.06,
                 stop_score=3.0, threads=2,
                 kws_threshold=0.12, kws_score=2.0, asr_precision="fp32",
                 endpoint_silence=1.0, command_timeout=8.0, no_speech_timeout=5.0, preroll=0.5):
        import sherpa_onnx as so

        self.stop_labels = {p.strip().upper().replace(" ", "_") for p in stop_phrases if p.strip()}
        # stop gets its own, more sensitive settings: a missed stop is worse than a false one
        kw_file = build_keywords_file(kws_dir, list(wake_phrases),
                                      extra_groups=[(list(stop_phrases), stop_score, stop_threshold)])
        q = ".int8"                          # the keyword model is tiny; int8 is fine there
        self.kws = so.KeywordSpotter(
            tokens=os.path.join(kws_dir, "tokens.txt"),
            encoder=_find(kws_dir, "encoder", q), decoder=_find(kws_dir, "decoder", q),
            joiner=_find(kws_dir, "joiner", q), keywords_file=kw_file, num_threads=1,
            keywords_score=kws_score, keywords_threshold=kws_threshold)
        # the small bilingual model loses words in int8, so fp32 is the default there
        enc = [f for f in os.listdir(asr_dir) if f.startswith("encoder") and f.endswith(".onnx")]
        want_int8 = asr_precision == "int8"
        pick = lambda part: next(os.path.join(asr_dir, f) for f in sorted(os.listdir(asr_dir))
                                 if f.startswith(part) and f.endswith(".onnx")
                                 and (".int8." in f) == want_int8)
        if not enc:
            raise FileNotFoundError(f"no ASR model in {asr_dir}")
        self.asr = so.OnlineRecognizer.from_transducer(
            tokens=os.path.join(asr_dir, "tokens.txt"), encoder=pick("encoder"),
            decoder=pick("decoder"), joiner=pick("joiner"), num_threads=threads,
            enable_endpoint_detection=True, rule1_min_trailing_silence=no_speech_timeout,
            rule2_min_trailing_silence=endpoint_silence, rule3_min_utterance_length=command_timeout)
        self._fresh_kws()
        self.kws_samples = 0
        self.asr_stream = None
        self.command_timeout = command_timeout
        self.no_speech_timeout = no_speech_timeout
        self.preroll = deque(maxlen=max(4, int(max(preroll, 0.8) * RATE / 400)))  # recent chunks
        self.cmd_samples = 0
        self.last_partial = ""
        self.wake_enabled = True

    def _fresh_kws(self):
        """New keyword stream, primed with a little near-silence: like the recognizer, a fresh
        stream misses words that start on its very first frames."""
        self.kws_stream = self.kws.create_stream()
        self.kws_stream.accept_waveform(RATE, np.random.normal(0, 1e-4, int(0.3 * RATE)).astype(np.float32))
        self.kws_samples = 0

    @property
    def capturing(self):
        return self.asr_stream is not None

    def _recent_audio(self, seconds):
        if not self.preroll:
            return np.zeros(0, np.float32)
        a = np.concatenate(list(self.preroll))
        return a[-int(seconds * RATE):]

    def _after_wake_gap(self, lookback=0.7):
        """Audio from the quietest moment in the last `lookback` seconds onward. The spotter
        reports the wake word ~0.2-0.35 s after it ends, when the next word has already begun;
        the quietest point is usually the little gap between the two."""
        a = self._recent_audio(lookback)
        hop = int(0.02 * RATE)
        if len(a) < 4 * hop:
            return a
        n = len(a) // hop
        env = np.sqrt(np.mean(a[:n * hop].reshape(n, hop) ** 2, axis=1))
        # ignore the final 60 ms (the gap must be before what we are hearing right now)
        cut = int(np.argmin(env[:max(1, n - 3)]))
        return a[cut * hop:]

    def start_command(self, preroll="followup"):
        """Open a command window. preroll='wake' starts at the gap after the wake word,
        'followup' includes the last half second (in case the person started early),
        'none' starts from now."""
        self.asr_stream = self.asr.create_stream()
        # the streaming recognizer drops speech that starts on its very first frames, so prime
        # it with a moment of near-silence before any real audio
        self.asr_stream.accept_waveform(RATE, np.random.normal(0, 1e-4, int(0.3 * RATE)).astype(np.float32))
        if preroll == "wake":
            head = self._after_wake_gap()
        elif preroll == "followup":
            head = self._recent_audio(0.5)
        else:
            head = np.zeros(0, np.float32)
        if len(head):
            self.asr_stream.accept_waveform(RATE, head)
        self.cmd_samples = 0
        self.last_partial = ""

    def feed(self, chunk):
        """Feed float32 mono 16 kHz audio. Returns a list of events:
           ('stop', label) | ('wake', label) | ('partial', text) | ('command', text)

        The keyword spotter runs all the time, so 'stop' is heard even while a command is being
        captured or while BETL is talking (wake_enabled only gates the wake word)."""
        chunk = np.asarray(chunk, dtype=np.float32)
        events = []
        self.preroll.append(chunk)
        # A keyword stream left running for a long time collects stale state and starts missing
        # words, so start a fresh one every few seconds, but only in a quiet moment so a word is
        # never cut in half.
        self.kws_samples += len(chunk)
        if self.kws_samples > 8 * RATE and float(np.sqrt(np.mean(chunk * chunk))) < 0.01:
            self._fresh_kws()
            self.kws_samples = 0
        self.kws_stream.accept_waveform(RATE, chunk)
        while self.kws.is_ready(self.kws_stream):
            self.kws.decode_stream(self.kws_stream)
            label = self.kws.get_result(self.kws_stream)
            if not label:
                continue
            self._fresh_kws()
            self.kws_samples = 0
            if label in self.stop_labels:
                events.append(("stop", label))
                self.asr_stream = None          # drop any half-heard command
                return events
            if self.asr_stream is None and self.wake_enabled:
                events.append(("wake", label))
                self.start_command(preroll="wake")
                return events

        if self.asr_stream is None:
            return events
        st = self.asr_stream
        st.accept_waveform(RATE, chunk)
        self.cmd_samples += len(chunk)
        while self.asr.is_ready(st):
            self.asr.decode_stream(st)
        text = self.asr.get_result(st).strip()
        if text != self.last_partial:
            self.last_partial = text
            events.append(("partial", text))
        if self.asr.is_endpoint(st) or self.cmd_samples > (self.command_timeout + 1) * RATE:
            events.append(("command", text))
            self.asr_stream = None
            self._fresh_kws()
            self.kws_samples = 0
        return events
