#!/usr/bin/env python3
"""BETL setup checker. Run on the X5 as the user BETL will run as:

    python3 scripts/betl_doctor.py            # check everything, blink the ring, say hello
    python3 scripts/betl_doctor.py --quiet    # no sound, no LED animation

Every line is PASS / WARN / FAIL, and every FAIL says what to do about it. Nothing here needs ROS
to be running (or the BETL service to be stopped... see the note printed if the devices are busy).
"""

import argparse
import glob
import grp
import importlib
import os
import pwd
import shutil
import subprocess
import sys
import tempfile
import time
import wave

HOME = os.path.expanduser("~")
MODELS = os.path.join(HOME, "betl_models")
WS = os.path.join(HOME, "betl_ws")
NEEDED_MODELS = [
    "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01",
    "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16",
    "vits-piper-zh_CN-huayan-medium",
    "argos/translate-en_zh-1_9",
    "argos/translate-zh_en-1_9",
]

results = []
USE_COLOR = sys.stdout.isatty()


def report(status, what, detail="", fix=""):
    colors = {"PASS": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m", "INFO": "\033[36m"}
    tag = f"{colors[status]}{status}\033[0m" if USE_COLOR else status
    print(f"[{tag}] {what}" + (f": {detail}" if detail else ""))
    if fix and status in ("FAIL", "WARN"):
        print(f"       -> {fix}")
    results.append(status)


def section(name):
    print(f"\n== {name}")


def run(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, str(e)


# ---------------------------------------------------------------------------------------------
def check_system():
    section("System")
    try:
        model = open("/proc/device-tree/model").read().strip("\x00\n ")
        report("PASS" if "X5" in model.upper() else "INFO", "board", model)
    except OSError:
        report("INFO", "board", "not an RDK board (or no device tree); continuing")
    user = pwd.getpwuid(os.getuid()).pw_name
    report("INFO", "running as", f"{user} (home {HOME})")
    if user == "root":
        report("WARN", "running as root", "models, notes and pip packages go in /root",
               "fine if you run BETL as root too; otherwise log in as sunrise and rerun")
    groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem}
    groups.add(grp.getgrgid(pwd.getpwnam(user).pw_gid).gr_name)
    for g in ("audio", "dialout"):
        if g in groups or user == "root":
            report("PASS", f"in group {g}")
        else:
            report("FAIL", f"not in group {g}", "", "run scripts/install_deps.sh, then log out and back in")
    if os.path.exists("/etc/udev/rules.d/99-betl.rules"):
        report("PASS", "udev rule installed")
    else:
        report("FAIL", "udev rule missing", "", "sudo cp udev/99-betl.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger")


def check_ros():
    section("ROS 2 and the BETL workspace")
    ros = next((p for p in ("/opt/tros/humble/setup.bash", "/opt/ros/humble/setup.bash",
                            "/opt/ros/jazzy/setup.bash") if os.path.exists(p)), None)
    if ros:
        report("PASS", "ROS 2 found", ros)
    else:
        report("FAIL", "no ROS 2 setup.bash found", "", "install TROS/ROS 2 Humble on the X5")
    ws_setup = os.path.join(WS, "install", "setup.bash")
    if os.path.exists(ws_setup) and glob.glob(os.path.join(WS, "install", "betl_voice", "lib", "betl_voice", "*")):
        report("PASS", "betl_voice is built", ws_setup)
    else:
        report("FAIL", "betl_voice not built", "", f"cd {WS} && source {ros or '<ros setup.bash>'} && colcon build --symlink-install")


def check_python():
    section("Python packages")
    mods = [("numpy", "numpy"), ("scipy", "scipy"), ("sounddevice", "sounddevice"), ("usb", "pyusb"),
            ("serial", "pyserial"), ("sherpa_onnx", "sherpa-onnx"), ("ctranslate2", "ctranslate2"),
            ("sentencepiece", "sentencepiece"), ("pypinyin", "pypinyin")]
    for mod, pkg in mods:
        try:
            m = importlib.import_module(mod)
            ver = getattr(m, "__version__", "")
            if mod == "numpy" and ver.split(".")[0] >= "2":
                report("WARN", "numpy", f"{ver} (ROS 2 Humble expects numpy 1.x)", 'pip3 install --user "numpy<2"')
            else:
                report("PASS", pkg, ver)
        except Exception as e:
            report("FAIL", pkg, str(e).splitlines()[0][:90], "bash scripts/install_deps.sh")
    if shutil.which("espeak-ng"):
        report("PASS", "espeak-ng", run(["espeak-ng", "--version"])[1][:60])
    else:
        report("FAIL", "espeak-ng not installed", "", "sudo apt install espeak-ng")


def check_models():
    section(f"Models ({MODELS})")
    for m in NEEDED_MODELS:
        p = os.path.join(MODELS, m)
        if os.path.isdir(p) and os.listdir(p):
            report("PASS", m)
        else:
            report("FAIL", f"missing {m}", "", "bash scripts/download_models.sh")
    llm = os.path.join(MODELS, "llm")
    if os.path.exists(os.path.join(llm, "llama-server")) and os.path.exists(os.path.join(llm, "model.gguf")):
        report("PASS", "local LLM (optional)", os.path.realpath(os.path.join(llm, "model.gguf")))
    else:
        report("WARN", "local LLM not set up (optional)", "chat will use canned answers",
               "bash scripts/setup_llm.sh")
    kdir = os.path.join(HOME, "betl_knowledge")
    n = len(glob.glob(os.path.join(kdir, "**", "*.md"), recursive=True) +
            glob.glob(os.path.join(kdir, "**", "*.txt"), recursive=True))
    report("PASS" if n else "INFO", "knowledge folder", f"{n} file(s) in {kdir}" if n else
           f"empty; drop .md/.txt files in {kdir} (see examples/knowledge)")


def check_respeaker(quiet):
    section("reSpeaker Flex")
    code, out = run(["lsusb"])
    if "2886:" in out:
        line = next(l for l in out.splitlines() if "2886:" in l)
        report("PASS", "on USB", line.split("ID ", 1)[-1])
    else:
        report("FAIL", "not found on USB", "", "check the cable; use the XMOS USB-C port next to RST")
        return
    # control interface (direction of arrival)
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
        from betl_voice.xvf import XVF
        x = XVF()
        ver = ".".join(str(v) for v in x.read("VERSION"))
        doa, speech = x.doa()
        report("PASS", "control interface", f"firmware {ver}, direction {doa} deg, speech={speech}")
        x.close()
    except Exception as e:
        msg = str(e)
        fix = ("permission problem: install the udev rule, then unplug/replug the reSpeaker"
               if "Access denied" in msg or "Errno 13" in msg else
               "is the BETL service running? it holds the device: sudo systemctl stop betl")
        report("FAIL", "control interface", msg[:90], fix)
    # audio devices
    try:
        import numpy as np
        import sounddevice as sd
        from betl_voice import audio_io
    except Exception as e:
        report("FAIL", "audio libraries", str(e)[:90], "bash scripts/install_deps.sh")
        return
    code, out = run(["arecord", "-l"])
    card = next((l for l in out.splitlines() if "reSpeaker" in l or "XVF" in l), "")
    report("PASS" if card else "WARN", "ALSA capture card", card.strip() or "not listed by arecord -l")
    holders = run(["fuser", "-v"] + glob.glob("/dev/snd/pcmC*"))[1]
    if any(w in holders for w in ("pulseaudio", "pipewire")):
        report("WARN", "a desktop sound server has audio devices open",
               "it can grab the reSpeaker before BETL does",
               "if BETL can't open the mic/speaker: systemctl --user mask --now pulseaudio.socket pulseaudio.service (as the desktop user)")
    dev_in = audio_io.find_device("input")
    dev_out = audio_io.find_device("output")
    if dev_in is None:
        report("FAIL", "mic not visible to PortAudio", "", "unplug/replug; run: python3 -c \"import sounddevice; print(sounddevice.query_devices())\"")
    else:
        try:
            frames = []
            stream, rate, ch = audio_io.open_stream("input", dev_in, lambda d, f, t, s: frames.append(d.copy()))
            with stream:
                time.sleep(1.5)
            a = np.concatenate(frames)[:, 0] if frames else np.zeros(1)
            db = 20 * np.log10(float(np.sqrt(np.mean(a * a))) + 1e-9)
            report("PASS" if db > -90 else "WARN", "mic records", f"{sd.query_devices(dev_in)['name']}, "
                   f"{ch} ch @ {rate} Hz, level {db:.0f} dBFS" + (" (silent? talk while this runs)" if db < -70 else ""))
        except Exception as e:
            report("FAIL", "mic won't open", str(e)[:100],
                   "something else has it open (sudo systemctl stop betl, or a desktop sound server)")
    if dev_out is None:
        report("FAIL", "speaker output not visible to PortAudio", "", "check the reSpeaker USB connection")
    elif not quiet:
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav") as f:
                subprocess.run(["espeak-ng", "-v", "en-us+klatt", "-p", "30", "-s", "145", "-w", f.name,
                                "Hello. I am Beetle. Setup check."], check=True, capture_output=True)
                with wave.open(f.name) as w:
                    r = w.getframerate()
                    y = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768
            stream, rate, ch = audio_io.open_stream("output", dev_out, prefer_rate=r)
            y = audio_io.resample(y, r, rate) * 0.8
            with stream:
                stream.write(np.repeat(y[:, None], ch, axis=1).astype(np.float32))
            report("PASS", "speaker", f"played a test phrase on {sd.query_devices(dev_out)['name']} "
                   "(did you hear 'Hello, I am Beetle'?)")
        except Exception as e:
            report("FAIL", "speaker playback", str(e)[:100], "plug a speaker into the reSpeaker's JST or 3.5 mm jack")


def check_ring(quiet):
    section("LED ring (RP2350)")
    try:
        import serial
        from serial.tools import list_ports
    except Exception as e:
        report("FAIL", "pyserial", str(e)[:80], "sudo apt install python3-serial")
        return
    ports = [p for p in list_ports.comports() if p.vid == 0x2E8A]
    if not ports:
        report("FAIL", "RP2350 not found on USB serial", "",
               "plug it in; MicroPython must be flashed (a bare board shows up as a USB drive instead)")
        return
    port = ports[0].device
    try:
        s = serial.Serial(port, 115200, timeout=0.5)
        s.reset_input_buffer()
        s.write(b"\n?\n")
        reply = ""
        t0 = time.time()
        while time.time() - t0 < 2 and "BETL-RING" not in reply:
            reply += s.readline().decode(errors="ignore")
        if "BETL-RING" in reply:
            report("PASS", "ring firmware answers", f"{port}: {reply.strip().splitlines()[-1]}")
            if not quiet:
                for cmd in ["B 120", "M listen", "P 0"] + [f"G {i * 4}" for i in (1, 2, 3, 0)]:
                    s.write((cmd + "\n").encode())
                    time.sleep(0.9 if cmd.startswith("G") else 0.05)
                for i in range(16):
                    s.write(f"N {i} 1\n".encode())
                    time.sleep(0.05)
                s.write(b"M stop\n")
                time.sleep(1.5)
                s.write(b"M idle\n")
                report("PASS", "ring animation sent", "you should have seen: green dot going round, an orange sweep, then a red pulse")
        else:
            report("FAIL", "no answer from ring firmware", f"{port} said {reply.strip()[:60]!r}",
                   "copy firmware/rp2350_led_ring/main.py to the board as main.py (Thonny), check RING_PIN")
        s.close()
    except Exception as e:
        msg = str(e)
        fix = ("busy: stop the BETL service first (sudo systemctl stop betl) or close Thonny"
               if "busy" in msg.lower() or "Errno 16" in msg else
               "permission: add yourself to dialout (install_deps.sh) and log back in")
        report("FAIL", f"can't open {port}", msg[:90], fix)


def check_service():
    section("Start on boot")
    code, out = run(["systemctl", "is-enabled", "betl"])
    if out.strip() == "enabled":
        active = run(["systemctl", "is-active", "betl"])[1].strip()
        report("PASS" if active == "active" else "WARN", "betl service", f"enabled, {active}",
               "journalctl -u betl -n 50 to see why")
    else:
        report("INFO", "betl service", "not installed yet", "bash scripts/install_service.sh")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true", help="skip the sound and LED tests")
    args = ap.parse_args()
    if run(["systemctl", "is-active", "betl"])[1].strip() == "active":
        print("Note: the betl service is running and holds the mic, speaker and ring.\n"
              "      Stop it for a full check:  sudo systemctl stop betl")
    check_system()
    check_ros()
    check_python()
    check_models()
    check_respeaker(args.quiet)
    check_ring(args.quiet)
    check_service()
    fails, warns = results.count("FAIL"), results.count("WARN")
    print(f"\n{fails} problem(s), {warns} warning(s).")
    if not fails:
        print("Looks good. Try:  ros2 launch betl_voice betl.launch.py   then say 'Hey Beetle'.")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
