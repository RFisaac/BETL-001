"""
Speak & Spell style voice: re-synthesize any speech through a 10-pole LPC vocoder at 8 kHz.

The mid-70s TI speech chips (TMS5100 / TMS5220 family) stored speech as LPC frames:
every 25 ms a frame holds an energy level, a pitch period (or "unvoiced"), and 10
reflection coefficients, quantized to a handful of bits each. The chip rebuilt the sound
with a buzz or noise source driving a 10-stage lattice filter, interpolating the
parameters 8 times per frame, at 8000 samples per second. That coarse, buzzy rebuild is
the sound.

This module does the same round trip on audio from a modern neural TTS:
    neural TTS audio -> 8 kHz -> LPC analysis -> quantize like the chip -> resynthesize

It is cheap: a 3-second sentence is about 120 frames of order-10 math (a few ms of numpy).

Standalone use (writes a WAV, no ROS needed):
    python lpc_voice.py "Spell robot" --model-dir ~/betl_models/vits-piper-en_US-ryan-low -o out.wav
    python lpc_voice.py --input some_speech.wav -o out.wav
"""

import math

import numpy as np
from scipy.signal import butter, lfilter, lfiltic, resample_poly

LPC_RATE = 8000
FRAME = 200          # 25 ms at 8 kHz, like the chip
SUBFRAMES = 8        # parameter interpolation steps per frame, like the chip
ORDER = 10
PREEMPH = 0.9375
# bits per reflection coefficient k1..k10, per the TMS5220 frame layout
K_BITS = (5, 5, 4, 4, 4, 4, 4, 3, 3, 3)


def _levinson(r, order):
    """Autocorrelation -> reflection coefficients k[1..order] and prediction error."""
    a = np.zeros(order + 1)
    a[0] = 1.0
    k = np.zeros(order)
    err = r[0]
    if err <= 1e-12:
        return k, 0.0
    for i in range(1, order + 1):
        acc = r[i] + np.dot(a[1:i], r[i - 1:0:-1])
        ki = -acc / err
        ki = max(-0.999, min(0.999, ki))
        k[i - 1] = ki
        a_prev = a.copy()
        for j in range(1, i):
            a[j] = a_prev[j] + ki * a_prev[i - j]
        a[i] = ki
        err *= (1.0 - ki * ki)
    return k, max(err, 0.0)


def _k_to_a(k):
    """Reflection coefficients -> direct-form denominator [1, a1..ap] (step-up)."""
    a = np.array([1.0])
    for ki in k:
        a_ext = np.concatenate([a, [0.0]])
        a = a_ext + ki * a_ext[::-1]
    return a


# Quantizer tables: 2**bits levels per coefficient, trained (1-D k-means in the log-area-ratio
# domain) on ~4500 frames of English and Mandarin speech. Like the TI chip's ROM tables they are
# lopsided and dense where speech actually puts each coefficient.
K_TABLES = tuple(np.array(t) for t in (
    (-0.962, -0.935, -0.909, -0.878, -0.844, -0.811, -0.775, -0.739, -0.695, -0.655, -0.619, -0.580, -0.538, -0.489, -0.442, -0.395, -0.352, -0.302, -0.249, -0.192, -0.129, -0.055, 0.032, 0.131, 0.236, 0.358, 0.473, 0.580, 0.680, 0.769, 0.849, 0.919),
    (-0.704, -0.542, -0.412, -0.298, -0.190, -0.117, -0.051, 0.008, 0.052, 0.098, 0.140, 0.180, 0.213, 0.244, 0.281, 0.316, 0.347, 0.380, 0.412, 0.443, 0.472, 0.497, 0.531, 0.565, 0.604, 0.645, 0.689, 0.734, 0.782, 0.827, 0.870, 0.918),
    (-0.762, -0.661, -0.565, -0.474, -0.382, -0.287, -0.192, -0.098, -0.001, 0.103, 0.215, 0.324, 0.427, 0.526, 0.625, 0.737),
    (-0.607, -0.459, -0.335, -0.216, -0.112, -0.019, 0.064, 0.142, 0.221, 0.304, 0.389, 0.482, 0.587, 0.691, 0.793, 0.902),
    (-0.558, -0.426, -0.319, -0.233, -0.156, -0.084, -0.012, 0.056, 0.126, 0.202, 0.278, 0.350, 0.425, 0.508, 0.595, 0.696),
    (-0.442, -0.283, -0.151, -0.057, 0.026, 0.101, 0.172, 0.237, 0.301, 0.370, 0.438, 0.504, 0.574, 0.648, 0.723, 0.807),
    (-0.593, -0.434, -0.321, -0.216, -0.123, -0.034, 0.045, 0.118, 0.189, 0.255, 0.328, 0.398, 0.467, 0.549, 0.634, 0.728),
    (-0.380, -0.200, -0.050, 0.083, 0.223, 0.373, 0.529, 0.687),
    (-0.394, -0.256, -0.149, -0.046, 0.048, 0.146, 0.258, 0.434),
    (-0.287, -0.139, -0.040, 0.039, 0.115, 0.197, 0.297, 0.426),
))


def _quantize_k(k, unvoiced):
    """Snap each coefficient to its nearest table entry. Unvoiced frames keep only k1-k4,
    as on the chip."""
    q = np.empty_like(k)
    for i, table in enumerate(K_TABLES):
        if unvoiced and i >= 4:
            q[i] = 0.0
        else:
            q[i] = table[np.abs(table - k[i]).argmin()]
    return q


E_TOP = 0.15          # level 14 ~ loudest vowels of peak-normalized speech


def _quantize_energy(rms):
    """15 usable levels about 3 dB apart below E_TOP, level 0 = silence (16 codes, as on the chip)."""
    lvl = 14 + round(2 * math.log2(max(rms, 1e-9) / E_TOP))
    if lvl < 1:
        return 0.0
    return E_TOP * 2 ** ((min(lvl, 15) - 14) / 2)


def _pitch(buf, start, n, lo, hi):
    """Normalized cross-correlation pitch tracker. Compares buf[start:start+n] with copies of
    itself shifted by lo..hi samples. Returns (period, strength 0..1)."""
    x0 = buf[start:start + n]
    e0 = np.dot(x0, x0)
    if e0 < 1e-8:
        return 0, 0.0
    shifted = np.lib.stride_tricks.sliding_window_view(buf[start + lo:start + hi + n], n)
    num = shifted @ x0
    el = np.einsum("ij,ij->i", shifted, shifted)
    nccf = num / np.sqrt(e0 * el + 1e-12)
    best = float(np.max(nccf))
    # take the shortest lag that is nearly as good as the best one (avoids octave-low errors)
    lag = int(np.argmax(nccf >= 0.88 * best)) + lo
    return lag, best


def _excitation_pulse(buzz=0.0):
    """Voiced excitation, one pulse per pitch period. buzz=0 is a plain impulse (flat
    spectrum, the LPC filter supplies the vowel colour). buzz>0 adds a short ringing tail
    that makes the tone harsher and more 'chip'-like."""
    if buzz <= 0:
        return np.array([1.0])
    n = np.arange(12)
    tail = np.cos(np.pi * 0.55 * n) * np.exp(-n / 2.5) * buzz
    pulse = np.concatenate([[1.0], tail[1:]])
    return pulse / np.sqrt(np.sum(pulse ** 2))


_PULSE = _excitation_pulse()


def analyze(x8k):
    """8 kHz float audio -> list of frames (energy, period, k[10]); period 0 = unvoiced."""
    x8k = np.asarray(x8k, dtype=np.float64)
    peak = np.max(np.abs(x8k))
    if peak > 0:
        x8k = x8k / peak                                 # fixed level -> consistent energy codes
    pre = lfilter([1.0, -PREEMPH], [1.0], x8k)
    # pitch is tracked on a low-passed copy (voice fundamentals live below ~900 Hz)
    b_lp, a_lp = butter(4, 900 / (LPC_RATE / 2))
    lp = lfilter(b_lp, a_lp, x8k)
    n_frames = max(1, len(x8k) // FRAME)
    win_len = int(FRAME * 2)
    window = np.hamming(win_len)
    lo, hi = LPC_RATE // 400, LPC_RATE // 60
    padn = win_len
    pad = np.concatenate([np.zeros(padn), pre, np.zeros(padn + hi)])
    lp_pad = np.concatenate([np.zeros(padn), lp, np.zeros(padn + hi)])
    frames = []
    for f in range(n_frames):
        s = f * FRAME                                    # this frame plays samples s..s+FRAME
        c = padn + s + FRAME // 2                        # its centre, in padded coordinates
        seg = pad[c - win_len // 2:c + win_len // 2] * window
        r = np.array([np.dot(seg[:win_len - i], seg[i:]) for i in range(ORDER + 1)])
        r[0] *= 1.0001                                   # tiny white-noise floor for stability
        k, err = _levinson(r, ORDER)
        gain = math.sqrt(err / np.sum(window ** 2)) if err > 0 else 0.0
        period, strength = _pitch(lp_pad, c - FRAME // 2, FRAME, lo, hi)
        voiced = strength > 0.45 and gain > 1e-3
        frames.append([gain, period if voiced else 0, k])
    # smooth out single-frame voicing flips and octave jumps
    for i in range(1, len(frames) - 1):
        p0, p1, p2 = frames[i - 1][1], frames[i][1], frames[i + 1][1]
        if p1 == 0 and p0 and p2:
            frames[i][1] = (p0 + p2) // 2
        elif p1 and not p0 and not p2:
            frames[i][1] = 0
        elif p1 and p0 and p2:
            mid = sorted((p0, p1, p2))[1]
            if abs(p1 - mid) > 0.3 * mid:
                frames[i][1] = mid
    return frames


def synthesize(frames, pitch_scale=1.0, monotone=0.0, monotone_hz=110.0, bits=8, breath=0.0, seed=1):
    """Frames -> 8 kHz float audio, with chip-style quantization and interpolation."""
    rng = np.random.default_rng(seed)
    q = []
    for gain, period, k in frames:
        unvoiced = period == 0
        if not unvoiced:
            f0 = LPC_RATE / period * pitch_scale
            f0 = (1 - monotone) * f0 + monotone * monotone_hz
            period = int(round(min(159, max(15, LPC_RATE / f0))))   # chip's pitch range
        q.append((_quantize_energy(gain), period, _quantize_k(k, unvoiced)))

    sub = FRAME // SUBFRAMES
    out = np.zeros(len(q) * FRAME)
    zi = np.zeros(ORDER)
    phase = 0                     # samples since last pulse
    prev_e, prev_p, prev_k = q[0]
    for fi, (e, p, k) in enumerate(q):
        # interpolate only when voicing doesn't change (the chip does the same)
        interp = (prev_p == 0) == (p == 0) and prev_e > 0 and e > 0
        for s in range(SUBFRAMES):
            t = (s + 1) / SUBFRAMES if interp else 1.0
            ee = prev_e + (e - prev_e) * t
            kk = prev_k + (k - prev_k) * t
            pp = int(round(prev_p + (p - prev_p) * t)) if (interp and p and prev_p) else p
            exc = np.zeros(sub)
            if ee <= 0:
                pass
            elif pp == 0:
                exc = rng.choice((-1.0, 1.0), sub) * ee
            else:
                amp = ee * math.sqrt(pp) * math.sqrt(1 - breath ** 2)
                for n in range(sub):
                    if phase >= pp:
                        phase = 0
                    if phase < len(_PULSE):
                        exc[n] = _PULSE[phase] * amp
                    phase += 1
                if breath:
                    exc += rng.choice((-1.0, 1.0), sub) * ee * breath
            a = _k_to_a(kk)
            # rebuild the filter memory for the *new* coefficients from the last outputs,
            # so coefficient changes don't inject a click every sub-frame
            i0 = fi * FRAME + s * sub
            past = out[max(0, i0 - ORDER):i0][::-1]
            zi = lfiltic([1.0], a, past) if len(past) else np.zeros(ORDER)
            y, _ = lfilter([1.0], a, exc, zi=zi)
            out[i0:i0 + sub] = y
        prev_e, prev_p, prev_k = e, p, k

    out = lfilter([1.0], [1.0, -0.9 * PREEMPH], out)          # undo the pre-emphasis (slightly thin)
    peak = np.max(np.abs(out))
    if peak > 0:
        out = out / peak * 0.9
    if bits:
        levels = 2 ** (bits - 1)
        out = np.round(out * levels) / levels
    return out.astype(np.float32)


def speak_and_spell(audio, rate, out_rate=None, pitch_scale=1.0, monotone=0.25, bits=8, breath=0.0):
    """Any-rate float audio -> Speak & Spell style audio at out_rate (default: same as input)."""
    out_rate = out_rate or rate
    g = math.gcd(int(rate), LPC_RATE)
    x8 = resample_poly(np.asarray(audio, dtype=np.float64), LPC_RATE // g, int(rate) // g)
    y8 = synthesize(analyze(x8), pitch_scale=pitch_scale, monotone=monotone, bits=bits, breath=breath)
    if out_rate == LPC_RATE:
        return y8
    g = math.gcd(int(out_rate), LPC_RATE)
    return resample_poly(y8, int(out_rate) // g, LPC_RATE // g).astype(np.float32)


def simple_robot(audio, rate, out_rate=None, bits=6):
    """Cheap fallback: band-limit to 8 kHz sampling and crush the bits."""
    out_rate = out_rate or rate
    g = math.gcd(int(rate), LPC_RATE)
    x8 = resample_poly(np.asarray(audio, dtype=np.float64), LPC_RATE // g, int(rate) // g)
    levels = 2 ** (bits - 1)
    x8 = np.round(x8 / (np.max(np.abs(x8)) + 1e-9) * 0.9 * levels) / levels
    g = math.gcd(int(out_rate), LPC_RATE)
    return resample_poly(x8, int(out_rate) // g, LPC_RATE // g).astype(np.float32)


def main():
    import argparse
    import time
    import wave

    ap = argparse.ArgumentParser(description="Speak & Spell style voice")
    ap.add_argument("text", nargs="?", help="text to speak (needs --model-dir)")
    ap.add_argument("--model-dir", help="sherpa-onnx piper voice folder")
    ap.add_argument("--input", help="process an existing WAV instead of text")
    ap.add_argument("-o", "--output", default="speak_and_spell.wav")
    ap.add_argument("--pitch", type=float, default=1.0, help="pitch scale")
    ap.add_argument("--monotone", type=float, default=0.25, help="0 = natural intonation, 1 = flat")
    ap.add_argument("--speed", type=float, default=0.85)
    ap.add_argument("--style", choices=["lpc", "simple", "clean"], default="lpc")
    args = ap.parse_args()

    if args.input:
        with wave.open(args.input) as w:
            rate = w.getframerate()
            raw = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768
            if w.getnchannels() > 1:
                raw = raw.reshape(-1, w.getnchannels())[:, 0]
        audio = raw
    else:
        import glob
        import os
        import sherpa_onnx
        d = args.model_dir
        onnx = [f for f in glob.glob(os.path.join(d, "*.onnx"))][0]
        tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=onnx, tokens=os.path.join(d, "tokens.txt"),
                    data_dir=os.path.join(d, "espeak-ng-data")),
                num_threads=2)))
        t0 = time.time()
        g = tts.generate(args.text, sid=0, speed=args.speed)
        print(f"TTS: {time.time() - t0:.3f} s")
        audio, rate = np.array(g.samples, dtype=np.float32), g.sample_rate

    t0 = time.time()
    if args.style == "lpc":
        y = speak_and_spell(audio, rate, pitch_scale=args.pitch, monotone=args.monotone)
    elif args.style == "simple":
        y = simple_robot(audio, rate)
    else:
        y = audio
    print(f"effect: {time.time() - t0:.3f} s for {len(audio) / rate:.2f} s of audio")
    with wave.open(args.output, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((np.clip(y, -1, 1) * 32767).astype(np.int16).tobytes())
    print("wrote", args.output)


if __name__ == "__main__":
    main()
