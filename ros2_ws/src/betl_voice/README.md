# betl_voice

The voice system for BETL, as a ROS 2 package for the RDK X5. Everything runs offline on the
robot, with no cloud services, so it works in China without a VPN.

**What BETL does in this prototype**

| Say | BETL does |
|---|---|
| "Beetle" / "Hey Beetle" | LED dot glides to your direction (stand-in for turning its head) and listens |
| "Beetle, please translate where is the train station" | says 火车站在哪？ |
| "Beetle, 请翻译 我们明天几点出发" | says "What time do we leave tomorrow?" |
| "Beetle, translate" … (pause) | asks "What should I translate?" and listens again, no wake word needed |
| **"Stop"** (no wake word needed, works mid-sentence) | cuts speech, ring pulses red, `/betl/estop` goes true, then ignores everything except… |
| "Beetle, resume" / "continue" / "all clear" / 继续 | estop false, back to normal |
| "Beetle, save a note to buy batteries" / "remember to call mom" / 记一下… | appends to `~/betl_notes/notes.md`, reads it back |
| "Beetle, take a note" … (pause) | asks "What should I note?" |
| "Beetle, read my notes" / "delete that note" | reads the latest three / removes the newest |
| "Beetle, look up the Beijing hotel" / "what did I note about…" / "search my notes for…" / 查一下… | searches your notes plus every .md/.txt in `~/betl_knowledge` and reads the best match |
| "Beetle, switch voices" / "use the robot voice" / "use the clear voice" / 换个声音 | swaps the Chinese voice and says 你好，我是比特尔 in it |
| anything else ("Beetle, when does the car pick us up?") | chat: a small local LLM (if started) answers, using any relevant notes/files; otherwise a few canned answers |

- **Voice:** espeak-ng's "klatt" (DECtalk-style) robot voice for English. Chinese uses a clear
  neural voice by default, or the klatt voice in Mandarin. An optional Speak & Spell LPC effect
  can go on any voice.
- **LED ring:** one dot always lit where BETL is "facing". Orange blips fade at the direction
  of other sounds. Separate looks for listening, thinking, speaking and stop.

**About stop.** Stop is spotted continuously by the same keyword spotter as the wake word, with
its own, more sensitive setting (a missed stop is worse than a false one). The brain publishes
`/betl/estop` (std_msgs/Bool) latched, plus a 5 Hz heartbeat. Your motion code should halt when
it is `true` *and* when the heartbeat goes quiet, so a crashed voice system also means stopped.
It is one more safety layer, not a replacement for the controller's dead-man switch.

---

## How it fits together

```
 reSpeaker Flex ──USB audio──► listener_node ──wake──────────► respeaker_node ──wake_direction──┐
   (XVF3800)   ──USB control─► respeaker_node ──doa──────────────────────────────────────────┐  │
                                listener_node ──command (JSON)──► brain_node ──say──► tts_node │  │
                                listener_node ──level (dBFS)──────────────────────────────┐   │  │
                                brain_node ────state──────────────────────────────────────┤   │  │
                                tts_node ──────speaking / speech_level ───────────────────┤   │  │
                                                                                          ▼   ▼  ▼
                                                                    led_node ──USB serial──► RP2350 + 16 LEDs
 tts_node ──USB audio──► reSpeaker speaker out  (its echo canceller uses this as the reference)
 listener_node ──stop──► brain_node ──estop (latched, 5 Hz)──► your motion stack
                         brain_node ──HTTP──► llama-server (optional local LLM)
```

All topics live in the `/betl` namespace and use plain `std_msgs`, so there is no message
package to build.

| Topic | Type | From → To | Meaning |
|---|---|---|---|
| `doa` | Float32 | respeaker → led | direction of the loudest sound, 0–359° (reSpeaker's own angle), 20 Hz |
| `voice_activity` | Bool | respeaker → | the XVF3800's speech detector |
| `wake` | String | listener → respeaker, brain | wake phrase heard |
| `wake_direction` | Float32 | respeaker → led (and later, the body) | where the wake word came from: the circular mean of the speech DoA over the last 1.5 s |
| `partial` | String | listener → | words so far, for display |
| `command` | String (JSON) | listener → brain | `{"text": "...", "lang": "en"\|"zh"}` after you pause |
| `level` | Float32 | listener → led | mic loudness in dBFS |
| `listen` | String | brain → listener | open a command window without the wake word (follow-up questions) |
| `say` | String (JSON) | brain → tts | `{"text": "...", "lang": "en"\|"zh"}` |
| `state` | String | brain → led | `idle`, `listen`, `think`, `stop` |
| `speaking`, `speech_level` | Bool, Float32 | tts → listener, brain, led | while talking, and how loud |
| `voice_mode` | String | brain → tts | `main`, `alt` or `toggle`: which Chinese voice to use |
| `stop` | Bool | listener, brain → tts, brain | "stop" heard: cut speech now and clear the queue |
| `estop` | Bool, latched + 5 Hz | brain → motion stack | `true` = halt. Treat a missing heartbeat as halt too |
| `transcript` | String | brain → | one readable line per exchange |

To make the robot body turn toward the speaker later, subscribe to `/betl/wake_direction`.

---

## Setup on the X5

**1. Wiring.** Plug the reSpeaker Flex into the X5 over USB (USB firmware, as it shipped).
Plug the RP2350-Zero in over USB. The ring's DIN goes to GPIO 0 (change `RING_PIN` at the
top of the firmware if you use another pin), plus 5 V and GND. Sixteen WS2812s at full white
draw about 1 A. The default brightness cap of 120/255 keeps it well under that.

**2. LED firmware.** Install MicroPython on the RP2350-Zero: hold BOOT, plug it in, and drag
on the RP2350 UF2 from micropython.org (the Pico 2 build works). Then in Thonny open
`firmware/rp2350_led_ring/main.py` and use *Save as → Raspberry Pi Pico → main.py*. On reboot
it does a quick orange sweep. Typing `?` in Thonny's shell should answer `BETL-RING 1 16`.

**3. Dependencies, permissions and models** (do the model download before the trip;
GitHub can be slow or blocked in China):
```bash
mkdir -p ~/betl_ws/src && cp -r betl_voice ~/betl_ws/src/
cd ~/betl_ws/src/betl_voice
bash scripts/install_deps.sh        # apt + pip packages, udev rule, audio/dialout groups
bash scripts/download_models.sh     # ~600 MB download, ~420 MB on disk, into ~/betl_models
```
Unplug and replug both USB devices, and log out and back in once.

**4. Build and run:**
```bash
cd ~/betl_ws
source /opt/ros/humble/setup.bash   # or your TROS setup script
colcon build --symlink-install
source install/setup.bash
ros2 launch betl_voice betl.launch.py          # add leds:=false to run without the ring
```
In another terminal, `ros2 topic echo /betl/transcript` shows what it heard and did.

**Optional: chat with a local LLM.** `bash scripts/setup_llm.sh` builds llama.cpp and downloads
Qwen 2.5 0.5B (or `setup_llm.sh 1.5b` for a smarter, ~3× slower model). Then launch with
`ros2 launch betl_voice betl.launch.py llm:=true`. Without it, chat falls back to canned
answers, and everything else works the same.

**Your own files.** Put a travel agenda, packing list or wiki pages (as .md or .txt) in
`~/betl_knowledge/`. `examples/knowledge/` shows the format; headings help BETL say where an
answer came from. Files are re-read automatically when they change.

**Check everything.** `python3 scripts/betl_doctor.py` tests every piece: groups and udev
rule, ROS, Python packages, models, the reSpeaker (USB, direction readout, mic level, a spoken
test phrase) and the LED ring (handshake and a test animation). Every failure prints the fix.

**Start on boot.** `bash scripts/install_service.sh` installs a `betl` systemd service that
runs as you, waits a few seconds for the USB devices, and restarts BETL if it crashes. It turns
on the local LLM if `setup_llm.sh` has been run.
- `sudo systemctl start|stop|restart betl` controls it. Restart after editing the config.
- `journalctl -u betl -f` shows the live log.
- Stop the service before running the doctor or Thonny, because it holds the mic, speaker
  and ring.

**5. Point the ring the right way.** Stand in front of LED 0 and say "Hey Beetle". The log
line `wake 'HEY_BEETLE' from 213 deg` gives you the angle. Put that number in `led0_deg` in
`config/betl.yaml`. If the dot then moves the wrong way round when you walk around it, set
`reverse: true`.

---

## Settings worth knowing (`config/betl.yaml`)

- **Wake word sensitivity:** `kws_threshold` (lower triggers more easily) and `kws_score`.
  Add spellings to `wake_phrases` if it misses how you say it. No retraining is needed; the
  phrases are turned into tokens at start-up.
- **Command timing:** `endpoint_silence_s` is the pause that ends a command (1.0 s);
  `no_speech_timeout_s` is how long it waits after the wake word.
- **Voice:** a voice is either `espeak:<voice>` or a Piper model folder.
  - `en_voice: "espeak:en-us+klatt"` is the default; `espeak_pitch` (0–99, lower = deeper) and
    `espeak_wpm` (speaking rate). Other espeak variants worth trying: `+croak` (gravelly),
    `+klatt3`, `+m3`. List them with `espeak-ng --voices=variant`.
  - `zh_voice: "espeak:cmn-latn-pinyin+klatt"` makes Chinese match the English robot voice.
    The text goes through pinyin first, because espeak's own Chinese dictionary reads tone
    numbers out loud in English.
  - Piper alternative: `en_voice: "vits-piper-en_US-ryan-low"`. Spelled-out letters
    ("R. O. B. O. T.") are rewritten as "R, O, B, O, T." for every voice, because Piper mumbles
    letters written with dots.
  - `en_style` / `zh_style`: `clean`, `lpc` (Speak & Spell effect, tuned with `pitch`,
    `monotone`, `breath`) or `simple` (bit-crushed).
- **LEDs:** `noise_db` / `noise_full_db` set which sounds make blips; `brightness`.

## Trying things without the hardware

- Voice only: `espeak-ng -v en-us+klatt -p 30 -s 145 "Spell robot"` plays the default English voice directly.
  `ros2 run betl_voice lpc_voice --input any.wav -o out.wav` Speak-and-Spell-ifies a recording.
- Replay a recording instead of the mic: set `input_file` on the listener. This is great for
  tuning wake-word settings against the same audio.
- Save everything BETL says: set `save_dir` on tts. With no speaker attached it still acts out
  the timing, so the LEDs and follow-ups behave normally.
- `python3 test/pipeline_demo.py` runs wake word → recognition → router → translation → voice
  on synthesized speech, with no ROS involved. `python3 -m pytest test/` runs the router tests.

## Models (all offline)

| Job | Model | Notes |
|---|---|---|
| Wake word | sherpa-onnx keyword spotter (gigaspeech, 3.3 M params, int8) | open vocabulary, phrases set in config |
| Speech → text | sherpa-onnx streaming zipformer, small, bilingual zh+en (fp32) | the int8 version dropped words in testing |
| Translation | Argos Translate 1.9 en→zh and zh→en, run directly with CTranslate2 | about 0.05 s per sentence on a desktop CPU |
| English voice | espeak-ng `en-us+klatt`, pitch 30, 145 wpm | 0.02 s per sentence; Piper ryan-low is downloaded as a fallback |
| Chinese voice | Piper `zh_CN-huayan-medium` | or espeak `cmn-latn-pinyin+klatt` via pypinyin |
| Chat (optional) | Qwen 2.5 0.5B Instruct (Q4 GGUF) via llama.cpp | answers are cut to two sentences; without notes to lean on it can make things up |

Speed measured on a 2-core x86 sandbox: the 0.5B chat model answers in 1–3 s, recognition runs about 10× faster than real time,
voices about 20× faster, and the LPC effect takes about 0.2 s per 5 s of speech. The X5's
Cortex-A55 cores will be several times slower. That should still be comfortably real-time,
but it hasn't been measured on the X5 yet. Raise `threads` if recognition lags.

## Known limits

- Translation quality is good for short travel phrases and weaker on long or idiomatic
  sentences. For an upgrade, NLLB-200-distilled-600M also runs on CTranslate2 (slower, better).
- "The Beatles" wakes it. That's fair enough.
- The direction comes from the XVF3800's DoA. It is best for speech and less precise for short
  noises.

## Next steps

- **Turn toward the speaker:** subscribe to `/betl/wake_direction` in the body controller.
- **Verified users:** when there's a camera, gate commands in `brain_node.on_command` on a
  "user verified" topic.
- **Better chat:** try the 1.5B model, or one of D-Robotics' BPU-accelerated LLM demos for the
  X5. `chat.py` only needs an OpenAI-style `/v1/chat/completions` endpoint.
- **Smarter recall:** the keyword search is simple and fast. Swap `knowledge.py` for embedding
  search if questions start missing.
