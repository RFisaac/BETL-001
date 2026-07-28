#!/usr/bin/env python3
"""
deck_teleop.py — runs on the Steam Deck (desktop mode).

Reads the Deck's built-in controller via pygame and streams
{vx, wz, deadman} as JSON over UDP to the X5. No ROS on the Deck,
no DDS over Wi-Fi.

Setup on the Deck (Ubuntu, one time):
    sudo apt install python3-pygame
Run:
    python3 deck_teleop.py --host <X5_IP>
Normally launched by beetle_deck.sh, which also opens the HUD kiosk.

DEADMAN: you must HOLD the deadman button (default L1) to drive.
Release = zeros sent immediately. This is the soft deadman from the
design doc's outstanding-safety list — release-to-stop, independent
of any velocity reading.

If the Deck's controller is acting as a mouse in desktop mode, open
Steam > Settings > Controller and set desktop layout to gamepad, or
just launch this script from Game Mode as a non-Steam app.
"""

import argparse
import json
import os
import socket
import time

# Must be set BEFORE pygame imports SDL:
# - dummy video: no window; runs headless behind the kiosk browser
# - background events: keep reading the controller while the browser
#   window has focus (SDL otherwise drops unfocused joystick input)
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame  # noqa: E402

# ---- Tune these to taste ---------------------------------------------------
# MAX_V MUST match the robot's wheel-speed cap, or the top of the stick does
# nothing useful. The robot clamps each wheel to max_wheel_tps turns/s, so:
#
#     MAX_V = max_wheel_tps * 2*pi*wheel_radius
#           = 0.6 * 2*pi*0.0765 = 0.288 m/s
#
# It used to be 0.6 m/s against a 0.29 m/s cap, so everything above half stick
# saturated both wheels. With per-side clamping that also meant no steering at
# all; the robot now scales both sides together, but a matched MAX_V is what
# makes the stick proportional instead of dead above halfway.
#
# Override at runtime with --max-v / --max-w rather than editing this.
MAX_V = 0.288      # m/s forward at full stick
# Pure rotation needs MAX_W*track/2/circ = 0.47 t/s at 1.2 rad/s — under the
# 0.6 cap, so yaw does not saturate on its own.
MAX_W = 1.2        # rad/s yaw at full stick
EXPO = 0.4         # 0 = linear stick, 1 = full cubic (finer near center)
DEADZONE = 0.08
SEND_HZ = 30

# Axis / button numbers — VERIFIED on Isaac's Deck (Ubuntu, hid-steam):
# axis 0/1 left stick X/Y, axis 2/3 right stick X/Y, up = negative.
AXIS_FWD = 1       # left stick vertical (negated in code)
AXIS_TURN = 2      # right stick horizontal
BTN_DEADMAN = 17   # L4 back grip (L1=9 if you'd rather use the bumper)
# ---------------------------------------------------------------------------


def shape(x: float) -> float:
    """Deadzone + expo curve."""
    if abs(x) < DEADZONE:
        return 0.0
    x = (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1 if x > 0 else -1)
    return (1 - EXPO) * x + EXPO * (x ** 3)


def main():
    global MAX_V, MAX_W      # must precede any use, incl. argparse defaults
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="X5 IP address")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--max-v", type=float, default=MAX_V,
                    help=f"m/s at full stick (default {MAX_V}); "
                         "match the robot's max_wheel_tps * 2*pi*r")
    ap.add_argument("--max-w", type=float, default=MAX_W,
                    help=f"rad/s at full stick (default {MAX_W})")
    ap.add_argument("--identify", action="store_true",
                    help="print raw axis/button numbers and exit-on-Ctrl-C; "
                         "use this once to confirm AXIS_/BTN_ constants")
    args = ap.parse_args()
    MAX_V, MAX_W = args.max_v, args.max_w

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No controller found. Desktop mode grabbing it as "
                         "a mouse? See docstring.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}  axes={js.get_numaxes()} "
          f"buttons={js.get_numbuttons()}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    period = 1.0 / SEND_HZ

    if args.identify:
        print("Move sticks / press buttons. Ctrl-C to quit.")
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
            btns = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            print(f"\raxes={axes} pressed={btns}   ", end="")
            time.sleep(0.1)

    print(f"Streaming to {dest}. HOLD button {BTN_DEADMAN} to drive.")
    try:
        while True:
            pygame.event.pump()
            deadman = bool(js.get_button(BTN_DEADMAN))
            vx = shape(-js.get_axis(AXIS_FWD)) * MAX_V if deadman else 0.0
            wz = shape(-js.get_axis(AXIS_TURN)) * MAX_W if deadman else 0.0
            pkt = json.dumps({"vx": vx, "wz": wz,
                              "deadman": deadman, "t": time.time()})
            sock.sendto(pkt.encode(), dest)
            status = "DRIVE" if deadman else f"IDLE (hold btn {BTN_DEADMAN})"
            print(f"\r{status:<22}  vx={vx:+.2f} m/s  wz={wz:+.2f} rad/s   ",
                  end="")
            time.sleep(period)
    except KeyboardInterrupt:
        # burst of zeros on the way out
        for _ in range(5):
            sock.sendto(json.dumps(
                {"vx": 0.0, "wz": 0.0, "deadman": False,
                 "t": time.time()}).encode(), dest)
            time.sleep(0.02)
        print("\nStopped, zeros sent.")


if __name__ == "__main__":
    main()
