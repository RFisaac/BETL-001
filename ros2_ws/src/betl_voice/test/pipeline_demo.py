"""End-to-end check without ROS or hardware: synthesize spoken commands, run them through
wake word -> recognizer -> router -> translator -> voice, and save the replies as WAVs.
    python3 test/pipeline_demo.py [models_dir] [out_dir]
"""
import os, sys, time, wave
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from betl_voice import audio_io, router
from betl_voice.speech import SpeechEngine, RATE
from betl_voice.translator import Translator
from betl_voice.tts_engine import Voice, _load_piper

MD = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/betl_models")
OUT = sys.argv[2] if len(sys.argv) > 2 else "pipeline_out"
os.makedirs(OUT, exist_ok=True)

t0 = time.time()
eng = SpeechEngine(f"{MD}/sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01",
                   f"{MD}/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16")
tr = Translator(f"{MD}/argos"); tr.warm_up()
voice = Voice("espeak:en-us+klatt", "vits-piper-zh_CN-huayan-medium", models_dir=MD)
print(f"loaded everything in {time.time()-t0:.1f} s")

speakers = {"ryan": _load_piper(f"{MD}/vits-piper-en_US-ryan-low", 2),
            "danny": _load_piper(f"{MD}/vits-piper-en_US-ryan-low", 2),  # second "speaker" (same voice; only default models needed)
            "zh": _load_piper(f"{MD}/vits-piper-zh_CN-huayan-medium", 2)}
def spoken(spk, text):
    g = speakers[spk].generate(text, sid=0, speed=1.0)
    return audio_io.resample(np.array(g.samples, np.float32), g.sample_rate, RATE)
sil = lambda s: np.zeros(int(RATE * s), np.float32) + np.random.normal(0, 3e-4, int(RATE * s)).astype(np.float32)

scenes = [
    ("translate EN->ZH", [sil(1), spoken("ryan", "Beetle, please translate where is the train station."), sil(2)]),
    ("translate EN->ZH, other voice", [sil(1), spoken("danny", "Hey Beetle, translate how much does this cost?"), sil(2)]),
    ("translate ZH->EN", [sil(1), spoken("danny", "Hey Beetle,"), sil(0.2), spoken("zh", "请翻译，洗手间在哪里？"), sil(2)]),
    ("wake, then nothing", [sil(1), spoken("danny", "Hey Beetle."), sil(6)]),
    ("translate, pause, phrase", [sil(1), spoken("ryan", "Beetle, translate."), sil(2.5)]),
    ("follow-up phrase", [spoken("danny", "I would like two bowls of noodles."), sil(2)]),
    ("chat", [sil(1), spoken("ryan", "Beetle, what is your name?"), sil(2)]),
    ("no wake word", [sil(1), spoken("danny", "The weather is nice today and I like bees."), sil(2)]),
]

pending = None
for name, parts in scenes:
    audio = np.concatenate(parts)
    if pending == "translate":
        eng.start_command(preroll="followup")
    events = []
    t_proc = time.time()
    for i in range(0, len(audio), 800):
        for ev in eng.feed(audio[i:i + 800]):
            if ev[0] != "partial":
                events.append(ev)
    proc = time.time() - t_proc
    print(f"\n== {name}  ({len(audio)/RATE:.1f} s audio, processed in {proc:.2f} s)")
    for kind, val in events:
        print(f"   {kind}: {val!r}")
    cmds = [v for k, v in events if k == "command"]
    if not cmds:
        continue
    text = cmds[-1]
    if pending == "translate":
        intent = router.Intent("translate", router.normalize(text), text); pending = None
    else:
        intent = router.parse(text)
    print(f"   intent: {intent.kind} {intent.text!r}")
    reply = None
    if intent.kind == "translate" and intent.text:
        t1 = time.time(); out, src, tgt = tr.translate(intent.text)
        print(f"   translation [{src}->{tgt}] {out!r} ({time.time()-t1:.2f} s)"); reply = (out, tgt)
    elif intent.kind == "translate":
        pending = "translate"; reply = ("What should I translate?", "en")
    elif intent.kind == "chat":
        from betl_voice.chat import canned; reply = canned(intent.text, "en")
    if reply:
        t1 = time.time(); y, sr = voice.synth(*reply)
        fn = f"{OUT}/{name.replace(' ', '_').replace(',', '').replace('>', '')}.wav"
        with wave.open(fn, "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
            w.writeframes((np.clip(y, -1, 1) * 32767).astype(np.int16).tobytes())
        print(f"   says [{reply[1]}] {reply[0]!r} -> {fn} (voice {time.time()-t1:.2f} s)")
