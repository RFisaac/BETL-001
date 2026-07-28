#!/usr/bin/env python3
"""
deck_tuner.py — Steam Deck tuning console for Beetle.

Drive-and-tune in one terminal: sticks drive, D-pad edits ODESC
parameters live, and per-wheel telemetry (with peak hold) comes back
from the X5 so you can see what a change actually did to current draw
and bus voltage.

Replaces deck_teleop.py for tuning sessions. Same UDP protocol plus a
`params` field going out and a telemetry stream coming back on
port+1.

Setup (Ubuntu on the Deck):
    sudo apt install python3-pygame
Run:
    python3 deck_tuner.py --host <X5_IP>

Requires odesc_teleop_node.py (X5) built for two-way UDP.

CONTROLS
    L4 (17) hold ............ deadman — wheels only move while held
    Left stick vert ......... forward / reverse
    Right stick horiz ....... turn
    D-pad up/down ........... select parameter
    D-pad left/right ........ adjust selected parameter -/+ 10%
    R1 (10) ................. reset peak-hold values
    Y (3) ................... revert SELECTED param to its saved value
    B (1) ................... clear errors + re-enter closed loop
    A (0) hold 5s ........... SAVE all four boards to flash
    X (2) hold 5s ........... reboot all drivers + reconnect
    Ctrl-C .................. quit (sends zeros)

The two 5-second holds (save, reboot) show a filling progress bar and
require the robot to be settled: sticks centred, deadman released,
wheels stopped. Move a stick and the hold aborts, telling you why.
Everything else is runtime-only — nothing persists until you save, so
a bad tuning session is undone by a power cycle.
"""

import argparse
import json
import os
import shutil
import socket
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame  # noqa: E402

# ---- Drive feel (Deck-side, not sent to the ODESCs) ------------------------
EXPO = 0.4
DEADZONE = 0.08
SEND_HZ = 30
UI_HZ = 10
# ---------------------------------------------------------------------------

AXIS_FWD = 1
AXIS_TURN = 2
BTN_DEADMAN = 17    # L4
BTN_DPAD_UP = 11
BTN_DPAD_DOWN = 12
BTN_DPAD_LEFT = 13
BTN_DPAD_RIGHT = 14
BTN_RESET_PEAKS = 10   # R1
BTN_REVERT = 3         # Y  — tap, selected param only
BTN_CLEAR = 1          # B  — tap
BTN_SAVE = 0           # A  — hold
BTN_REBOOT = 2         # X  — hold

HOLD_SECONDS = 5.0     # how long A / X must be held
SETTLE_DEADZONE = 0.15 # stick movement that aborts a hold (drift-tolerant)
# A stationary wheel's velocity observer still reports +/-0.045 turns/s of
# noise (measured 2026-07-22), so a 0.05 threshold aborted holds constantly.
# 0.15 turns/s is ~8 cm/s at the wheel — clearly above noise, clearly below
# any motion you'd fail to notice.
STOPPED_TPS = 0.15
# ...and one noisy sample shouldn't kill a 5-second hold, so require the
# wheels to read moving this many polls in a row before aborting.
MOVING_SAMPLES = 3

# Parameter table. `key` is what the X5 node expects; `path` documents
# where it lives on the ODrive object. Bounds stop a stuck button from
# running a value to absurdity — they are not moral judgments.
PARAMS = [
    # key                    label                min     max    fmt
    ("vel_gain",            "vel_gain",           0.005,  3.0,   "{:.3f}"),
    ("vel_integrator_gain", "vel_integ_gain",     0.0,    2.0,   "{:.3f}"),
    ("dc_max_positive_current", "dc_max_pos_I",   1.0,   25.0,   "{:.2f}"),
    ("dc_max_negative_current", "dc_max_neg_I",  -25.0,  -0.5,   "{:.2f}"),
    ("current_lim",         "current_lim",        1.0,   60.0,   "{:.1f}"),
    ("vel_limit",           "vel_limit",          0.5,   30.0,   "{:.2f}"),
    ("torque_constant",     "torque_const",       0.01,   2.0,   "{:.4f}"),
    ("_max_v",              "MAX_V (deck)",       0.05,   3.0,   "{:.2f}"),
    ("_max_w",              "MAX_W (deck)",       0.1,    6.0,   "{:.2f}"),
]
STEP = 0.10   # proportional step per press

WHEEL_ORDER = ["left_front", "right_front", "left_rear", "right_rear"]
SHORT = {"left_front": "LF", "right_front": "RF",
         "left_rear": "LR", "right_rear": "RR"}


def shape(x):
    if abs(x) < DEADZONE:
        return 0.0
    x = (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1 if x > 0 else -1)
    return (1 - EXPO) * x + EXPO * (x ** 3)


class Tuner:
    def __init__(self, host, port):
        self.dest = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port + 1))   # telemetry return path
        self.sock.setblocking(False)

        self.sel = 0
        self.deck_params = {"_max_v": 0.6, "_max_w": 1.2}
        self.pending = {}        # key -> value we want applied
        self.applied = {}        # key -> value X5 reports as live
        self.saved = {}          # key -> value X5 reports as in flash
        self.disagree = set()    # keys where the four boards differ
        self.wheels = {}         # per-wheel telemetry from X5
        self.peaks = {}          # wheel -> peak |iq|
        self.vbus_min = None
        self.last_rx = 0.0
        self.log = []
        self.busy = ""           # non-empty while X5 reports a long op
        self.save_serial = 0     # bumps when the X5 finishes a save
        self.prev_buttons = set()
        self.hold_btn = None     # which button is being held
        self.hold_start = 0.0
        self.hold_label = ""
        self.moving_count = 0    # consecutive polls reading "wheels moving"
        self.fault_detail = []   # decoded error names for the newest fault

    # ---- comms ------------------------------------------------------------

    def send(self, vx, wz, deadman, command=None):
        pkt = {"vx": vx, "wz": wz, "deadman": deadman, "t": time.time()}
        if self.pending:
            pkt["params"] = {k: v for k, v in self.pending.items()
                             if not k.startswith("_")}
        if command:
            pkt["command"] = command
        self.sock.sendto(json.dumps(pkt).encode(), self.dest)

    def poll(self):
        newest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(8192)
                newest = data
            except BlockingIOError:
                break
            except OSError:
                break
        if newest is None:
            return
        try:
            d = json.loads(newest.decode())
        except ValueError:
            return
        self.last_rx = time.time()
        self.wheels = d.get("wheels", {})
        self.applied = d.get("applied", {})
        self.saved = d.get("saved", {})
        self.disagree = set(d.get("disagree", []))
        self.busy = d.get("busy", "")
        serial = d.get("save_serial", 0)
        if serial != self.save_serial:
            # a save just finished: flash is now the source of truth, and
            # anything still queued would re-dirty values we just committed
            self.save_serial = serial
            self.pending.clear()
        for msg in d.get("log", []):
            self.log.append(msg)
        self.log = self.log[-6:]
        # peak hold, computed Deck-side so resets are instant
        for w, info in self.wheels.items():
            iq = abs(info.get("iq", 0.0) or 0.0)
            self.peaks[w] = max(self.peaks.get(w, 0.0), iq)
            vb = info.get("vbus")
            if vb:
                self.vbus_min = vb if self.vbus_min is None \
                    else min(self.vbus_min, vb)

    # ---- input ------------------------------------------------------------

    def edges(self, js):
        """Return the set of buttons newly pressed this frame."""
        now = {i for i in range(js.get_numbuttons()) if js.get_button(i)}
        new = now - self.prev_buttons
        self.prev_buttons = now
        return now, new

    def current_value(self, key):
        if key.startswith("_"):
            return self.deck_params.get(key)
        if key in self.pending:
            return self.pending[key]
        return self.applied.get(key)

    def revert_selected(self):
        key = PARAMS[self.sel][0]
        label = PARAMS[self.sel][1]
        if key.startswith("_"):
            self.log.append(f"{label}: deck-side, nothing saved to revert to")
            return
        sav = self.saved.get(key)
        if sav is None:
            self.log.append(f"{label}: no saved value known")
            return
        self.pending[key] = sav
        self.log.append(f"{label} -> saved value {sav:g}")

    def adjust(self, key, direction):
        meta = next(p for p in PARAMS if p[0] == key)
        _, _, lo, hi, _ = meta
        v = self.current_value(key)
        if v is None:
            self.log.append(f"{key}: no value from X5 yet")
            return
        if abs(v) < 1e-9:
            # proportional steps can't escape zero — jump to the bound
            nv = lo if direction > 0 else v
        else:
            nv = v * (1 + STEP * direction)
        nv = max(lo, min(hi, nv))
        if key.startswith("_"):
            self.deck_params[key] = nv
        else:
            self.pending[key] = nv

    # ---- rendering --------------------------------------------------------

    def render(self, deadman, vx, wz):
        size = shutil.get_terminal_size((118, 28))
        cols, rows = size.columns, size.lines
        out = []
        link = (time.time() - self.last_rx) < 1.0
        state = "DRIVE" if deadman else "IDLE"
        head = (f" BEETLE TUNER   {state:<6} "
                f"vx={vx:+.2f} m/s  wz={wz:+.2f} rad/s   "
                f"X5:{'OK' if link else 'NO LINK'}")
        if self.busy:
            head += f"   >> {self.busy}"
        out.append("\033[7m" + head.ljust(cols)[:cols] + "\033[0m")

        # ---- left: parameters | right: log --------------------------------
        LEFT_W = 46
        left = ["  PARAMETER          LIVE        SAVED"]
        for i, (key, label, lo, hi, fmt) in enumerate(PARAMS):
            cur = self.current_value(key)
            sav = self.deck_params.get(key) if key.startswith("_") \
                else self.saved.get(key)
            curs = fmt.format(cur) if cur is not None else "  --  "
            savs = fmt.format(sav) if sav is not None else "  --  "
            dirty = (key in self.pending
                     and self.applied.get(key) is not None
                     and abs(self.pending[key]
                             - self.applied.get(key, 0)) > 1e-9)
            changed = (cur is not None and sav is not None
                       and abs(cur - sav) > 1e-9)
            mark = "*" if changed else " "
            flag = "!" if key in self.disagree else (
                "~" if dirty else " ")
            row = f"  {label:<16} {curs:>9} {mark} {savs:>9} {flag}"
            if i == self.sel:
                left.append("\033[1;36m>" + row[1:].ljust(LEFT_W - 1)
                            + "\033[0m")
            else:
                left.append(row.ljust(LEFT_W))

        right = ["  LOG"]
        for line in self.log[-(len(left) - 1):]:
            right.append(f"  \033[90m{line[:cols - LEFT_W - 4]}\033[0m")

        for i in range(max(len(left), len(right))):
            l = left[i] if i < len(left) else " " * LEFT_W
            r = right[i] if i < len(right) else ""
            # ljust on the raw string only when it carries no escape codes
            if "\033" not in l:
                l = l.ljust(LEFT_W)
            out.append(l + r)

        # ---- wheels --------------------------------------------------------
        out.append("  WHEEL   CMD    ACTUAL     Iq    PEAK    VBUS   STATE")
        self.fault_detail = []
        for w in WHEEL_ORDER:
            info = self.wheels.get(w)
            if not info:
                out.append(f"  {SHORT[w]:<6}   --      --       --     --"
                           f"      --     --")
                continue
            fault = info.get("fault")
            st = info.get("state", "?")
            stxt = "FAULT" if fault else ("CL" if st == 8 else f"st{st}")
            row = (f"  {SHORT[w]:<6} {info.get('cmd', 0):>5.2f} "
                   f"{info.get('vel', 0):>8.2f} "
                   f"{info.get('iq', 0) or 0:>7.2f} "
                   f"{self.peaks.get(w, 0):>7.2f} "
                   f"{info.get('vbus', 0) or 0:>7.1f}   {stxt}")
            if fault:
                err = info.get("err", "")
                names = info.get("err_names") or []
                root = next((n for n in names
                             if n.startswith(("ENCODER:", "MOTOR:"))),
                            names[0] if names else "")
                root = root.split(": ")[-1] if root else ""
                out.append("\033[31m" + row + f"  {err} {root}" + "\033[0m")
                self.fault_detail = [f"{SHORT[w]} {err}: "
                                     + ", ".join(n.split(': ')[-1]
                                                 for n in names)]
            else:
                out.append(row)
        vb = f"{self.vbus_min:.1f} V" if self.vbus_min is not None else "--"
        out.append(f"          vbus low-water: {vb}")

        for line in self.fault_detail:
            out.append(f"  \033[31m{line[:cols - 4]}\033[0m")

        # ---- hold bar sits just above the footer ---------------------------
        if self.hold_btn is not None:
            frac = min(1.0, (time.time() - self.hold_start) / HOLD_SECONDS)
            width = max(20, min(cols - 26, 60))
            filled = int(frac * width)
            bar = "█" * filled + "░" * (width - filled)
            remain = max(0.0, HOLD_SECONDS - (time.time() - self.hold_start))
            out.append(f"  \033[1;33m{self.hold_label:<7}\033[0m "
                       f"\033[33m{bar}\033[0m {remain:>3.1f}s")

        out.append("\033[90m  D-pad:select/adjust  R1:peaks  Y:revert sel"
                   "  B:clear  A(hold):save  X(hold):reboot\033[0m")

        # never overflow the screen — the footer is worth more than log tail
        if len(out) > rows - 1:
            out = out[:rows - 2] + [out[-1]]
        sys.stdout.write("\033[H\033[J" + "\n".join(out) + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=5005)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No controller found.")
    js = pygame.joystick.Joystick(0)
    js.init()

    t = Tuner(args.host, args.port)
    send_period = 1.0 / SEND_HZ
    ui_period = 1.0 / UI_HZ
    last_ui = 0.0
    sys.stdout.write("\033[2J")

    try:
        while True:
            pygame.event.pump()
            held, new = t.edges(js)
            t.poll()

            deadman = BTN_DEADMAN in held
            vx = shape(-js.get_axis(AXIS_FWD)) * t.deck_params["_max_v"] \
                if deadman else 0.0
            wz = shape(-js.get_axis(AXIS_TURN)) * t.deck_params["_max_w"] \
                if deadman else 0.0

            command = None
            raw_fwd = -js.get_axis(AXIS_FWD)
            raw_turn = -js.get_axis(AXIS_TURN)
            sticks_centred = (abs(raw_fwd) < SETTLE_DEADZONE
                              and abs(raw_turn) < SETTLE_DEADZONE)
            wheels_now = any(abs(w.get("vel", 0) or 0) > STOPPED_TPS
                             for w in t.wheels.values())
            t.moving_count = t.moving_count + 1 if wheels_now else 0
            wheels_moving = t.moving_count >= MOVING_SAMPLES

            # --- 5-second holds: A = save, X = reboot ---------------------
            hold_map = {BTN_SAVE: "SAVE", BTN_REBOOT: "REBOOT"}
            active_hold = next((b for b in hold_map if b in held), None)

            if active_hold is None:
                if t.hold_btn is not None:
                    t.log.append(f"{t.hold_label} cancelled (released)")
                t.hold_btn = None
            else:
                if t.hold_btn != active_hold:          # hold just started
                    t.hold_btn = active_hold
                    t.hold_label = hold_map[active_hold]
                    t.hold_start = time.time()
                abort = None
                if deadman:
                    abort = "deadman held"
                elif not sticks_centred:
                    abort = "stick moved"
                elif wheels_moving:
                    abort = "wheels still turning"
                elif t.busy:
                    abort = "X5 busy"
                if abort:
                    t.log.append(f"{t.hold_label} aborted: {abort}")
                    t.hold_btn = None
                elif time.time() - t.hold_start >= HOLD_SECONDS:
                    command = "save" if active_hold == BTN_SAVE else "reboot"
                    t.log.append(f"{t.hold_label} confirmed")
                    t.hold_btn = None

            # --- taps -----------------------------------------------------
            if t.hold_btn is None and command is None:
                if BTN_DPAD_UP in new:
                    t.sel = (t.sel - 1) % len(PARAMS)
                if BTN_DPAD_DOWN in new:
                    t.sel = (t.sel + 1) % len(PARAMS)
                if BTN_DPAD_LEFT in new:
                    t.adjust(PARAMS[t.sel][0], -1)
                if BTN_DPAD_RIGHT in new:
                    t.adjust(PARAMS[t.sel][0], +1)
                if BTN_RESET_PEAKS in new:
                    t.peaks.clear()
                    t.vbus_min = None
                    t.log.append("peaks reset")
                if BTN_CLEAR in new:
                    command = "clear"
                if BTN_REVERT in new:
                    t.revert_selected()

            t.send(vx, wz, deadman, command)

            # once the X5 confirms a param, stop resending it
            for k in list(t.pending):
                if (k in t.applied
                        and abs(t.applied[k] - t.pending[k]) < 1e-9):
                    del t.pending[k]

            now = time.time()
            if now - last_ui >= ui_period:
                t.render(deadman, vx, wz)
                last_ui = now
            time.sleep(send_period)

    except KeyboardInterrupt:
        for _ in range(5):
            t.send(0.0, 0.0, False)
            time.sleep(0.02)
        sys.stdout.write("\033[2J\033[H")
        print("Stopped, zeros sent.")


if __name__ == "__main__":
    main()