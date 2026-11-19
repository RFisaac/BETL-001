"""Finding and opening the reSpeaker's audio input/output with sounddevice (PortAudio)."""

import numpy as np

HINTS = ("respeaker", "xvf3800", "xvf")
TRY_RATES = (16000, 48000, 44100)


def _sd():
    import sounddevice as sd
    return sd


def find_device(kind, hint=""):
    """Index of the first device whose name contains `hint` (or a reSpeaker hint) and has
    `kind` ('input' or 'output') channels. Returns None if nothing matches."""
    sd = _sd()
    key = "max_input_channels" if kind == "input" else "max_output_channels"
    hints = [hint.lower()] if hint else list(HINTS)
    for i, d in enumerate(sd.query_devices()):
        if d[key] > 0 and any(h in d["name"].lower() for h in hints):
            return i
    return None


def open_stream(kind, device, callback=None, blocksize=0, prefer_rate=16000):
    """Open an input or output stream on `device`, trying sensible sample rates and
    channel counts until one works. Returns (stream, rate, channels)."""
    sd = _sd()
    info = sd.query_devices(device)
    max_ch = int(info["max_input_channels" if kind == "input" else "max_output_channels"])
    rates = [prefer_rate] + [r for r in TRY_RATES if r != prefer_rate] + [int(info["default_samplerate"])]
    chans = [max_ch] + ([2, 1] if max_ch > 2 else [1] if max_ch == 2 else [])
    cls = sd.InputStream if kind == "input" else sd.OutputStream
    last = None
    for rate in dict.fromkeys(rates):
        for ch in dict.fromkeys(chans):
            try:
                bs = blocksize or rate // 20
                stream = cls(device=device, samplerate=rate, channels=ch, dtype="float32",
                             blocksize=bs, callback=callback)
                return stream, rate, ch
            except Exception as e:      # PortAudio raises various errors for unsupported formats
                last = e
    raise RuntimeError(f"could not open {kind} on device {device}: {last}")


def resample(x, src, dst):
    if src == dst:
        return np.asarray(x, dtype=np.float32)
    from math import gcd
    from scipy.signal import resample_poly
    g = gcd(int(src), int(dst))
    return resample_poly(np.asarray(x, dtype=np.float64), int(dst) // g, int(src) // g).astype(np.float32)


def describe_devices():
    sd = _sd()
    lines = []
    for i, d in enumerate(sd.query_devices()):
        lines.append(f"[{i}] {d['name']}  in={d['max_input_channels']} out={d['max_output_channels']} "
                     f"rate={int(d['default_samplerate'])}")
    return "\n".join(lines)
