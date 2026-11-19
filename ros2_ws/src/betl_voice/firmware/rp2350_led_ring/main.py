# BETL LED ring firmware - MicroPython for the Waveshare RP2350-Zero
#
# Copy this file to the board as main.py (Thonny: File > Save as > Raspberry Pi Pico > main.py).
# It drives a 16-LED WS2812 ring and listens for one-line text commands on USB serial.
#
# Animation runs here, so the X5 only sends high-level events and the ring stays smooth even if
# the X5 is busy. Positions are in LED units (0.0 .. 16.0, fractional = between two LEDs).
#
# Commands (newline terminated):
#   ?                     -> replies "BETL-RING 1 <num_leds>"
#   P <pos>               set the resting dot position immediately
#   G <pos>               glide the dot to <pos> the short way round (the "turn to you" move)
#   N <pos> <0..1>        a noise blip at <pos> that fades out
#   M <mode>              idle | listen | think | speak | stop | off
#   L <0..1>              speech level, used by speak mode
#   B <0..255>            overall brightness cap
#   C <slot> <rrggbb>     colour for slot: home noise listen think speak stop
# Every accepted command replies "ok"; anything else replies "err <reason>".

import math
import sys
import time

import machine
import neopixel
import select

# ---- wiring ----------------------------------------------------------------------------
RING_PIN = 1           # GPIO (GP number on the board) wired to the ring's DIN
NUM_LEDS = 16
ONBOARD_PIN = 16       # the RP2350-Zero's own RGB LED (mirrors the mode colour, handy for debugging)
FPS = 50

ring = neopixel.NeoPixel(machine.Pin(RING_PIN), NUM_LEDS)
try:
    onboard = neopixel.NeoPixel(machine.Pin(ONBOARD_PIN), 1)
except Exception:
    onboard = None


def hex_rgb(h):
    h = h.strip().lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


COLORS = {
    "home": hex_rgb("00b4ff"),     # resting dot
    "noise": hex_rgb("ff7a00"),    # other sounds
    "listen": hex_rgb("00ffa0"),   # after the wake word
    "think": hex_rgb("8a5cff"),    # working (translating, etc.)
    "speak": hex_rgb("ffffff"),    # while talking
    "stop": hex_rgb("ff0000"),     # stopped / safety hold
}

# gamma table so fades look even to the eye
GAMMA = bytearray(int(((i / 255) ** 2.2) * 255 + 0.5) for i in range(256))


def wrap(p):
    return p % NUM_LEDS


def short_delta(a, b):
    """Signed shortest distance from a to b around the ring."""
    d = (b - a) % NUM_LEDS
    return d - NUM_LEDS if d > NUM_LEDS / 2 else d


def ease_in_out(t):
    return 4 * t * t * t if t < 0.5 else 1 - ((-2 * t + 2) ** 3) / 2


class Ring:
    def __init__(self):
        self.mode = "idle"
        self.pos = 0.0               # where the dot is drawn
        self.glide_from = 0.0
        self.glide_delta = 0.0
        self.glide_start = 0
        self.glide_ms = 0
        self.noise = [0.0] * NUM_LEDS
        self.level = 0.0
        self.brightness = 160
        self.t0 = time.ticks_ms()

    # -- commands --------------------------------------------------------------------------
    def handle(self, line):
        parts = line.split()
        if not parts:
            return None
        cmd = parts[0].upper()
        try:
            if cmd == "?":
                return "BETL-RING 1 %d" % NUM_LEDS
            if cmd == "P":
                self.pos = wrap(float(parts[1]))
                self.glide_ms = 0
            elif cmd == "G":
                target = wrap(float(parts[1]))
                self.glide_from = self.pos
                self.glide_delta = short_delta(self.pos, target)
                self.glide_start = time.ticks_ms()
                # longer turns take longer, like a body turning
                self.glide_ms = int(250 + 650 * abs(self.glide_delta) / (NUM_LEDS / 2))
            elif cmd == "N":
                p = wrap(float(parts[1]))
                s = max(0.0, min(1.0, float(parts[2]) if len(parts) > 2 else 1.0))
                i = int(p)
                frac = p - i
                self._bump(i, s * (1 - frac))
                self._bump(i + 1, s * frac)
            elif cmd == "M":
                m = parts[1].lower()
                if m not in ("idle", "listen", "think", "speak", "stop", "off"):
                    return "err mode"
                self.mode = m
                self.t0 = time.ticks_ms()
            elif cmd == "L":
                self.level = max(0.0, min(1.0, float(parts[1])))
            elif cmd == "B":
                self.brightness = max(0, min(255, int(parts[1])))
            elif cmd == "C":
                if parts[1] not in COLORS:
                    return "err slot"
                COLORS[parts[1]] = hex_rgb(parts[2])
            else:
                return "err cmd"
        except (IndexError, ValueError):
            return "err args"
        return "ok"

    def _bump(self, i, s):
        i %= NUM_LEDS
        self.noise[i] = min(1.0, max(self.noise[i], s))

    # -- animation ---------------------------------------------------------------------------
    def step(self, now):
        if self.glide_ms:
            t = time.ticks_diff(now, self.glide_start) / self.glide_ms
            if t >= 1:
                self.pos = wrap(self.glide_from + self.glide_delta)
                self.glide_ms = 0
            else:
                self.pos = wrap(self.glide_from + self.glide_delta * ease_in_out(t))
        for i in range(NUM_LEDS):
            self.noise[i] *= 0.93         # ~200 ms half-life at 50 fps

    def render(self, now):
        el = time.ticks_diff(now, self.t0) / 1000.0
        px = [[0.0, 0.0, 0.0] for _ in range(NUM_LEDS)]
        mode = self.mode

        def add(i, color, s):
            if s <= 0:
                return
            p = px[i % NUM_LEDS]
            p[0] += color[0] * s
            p[1] += color[1] * s
            p[2] += color[2] * s

        def dot(pos, color, s, width=1.0):
            # anti-aliased dot: light the LEDs within `width` of pos
            for i in range(NUM_LEDS):
                d = abs(short_delta(pos, i))
                if d < width:
                    add(i, color, s * (1 - d / width))

        if mode == "off":
            pass
        elif mode == "stop":
            s = 0.15 + 0.85 * (0.5 + 0.5 * math.sin(el * 2 * math.pi * 1.2))
            for i in range(NUM_LEDS):
                add(i, COLORS["stop"], s)
        else:
            # other sounds: faint blips that fade
            for i in range(NUM_LEDS):
                add(i, COLORS["noise"], self.noise[i])
            if mode == "idle":
                dot(self.pos, COLORS["home"], 0.6)
            elif mode == "listen":
                breathe = 0.7 + 0.3 * math.sin(el * 2 * math.pi * 0.8)
                dot(self.pos, COLORS["listen"], 1.0 * breathe, width=1.6)
            elif mode == "think":
                dot(self.pos, COLORS["home"], 0.5)
                for k in (0, NUM_LEDS / 2):
                    dot(self.pos + k + el * 10, COLORS["think"], 0.8, width=1.2)
            elif mode == "speak":
                dot(self.pos, COLORS["speak"], 0.25 + 0.75 * self.level, width=1.0 + self.level)

        scale = self.brightness / 255
        for i in range(NUM_LEDS):
            r, g, b = (min(255, int(c * scale)) for c in px[i])
            ring[i] = (GAMMA[r], GAMMA[g], GAMMA[b])
        ring.write()
        if onboard:
            c = {"stop": COLORS["stop"], "listen": COLORS["listen"], "think": COLORS["think"],
                 "speak": COLORS["speak"]}.get(mode, (0, 0, 0) if mode == "off" else COLORS["home"])
            onboard[0] = tuple(GAMMA[int(v * 0.15)] for v in c)
            onboard.write()


def main():
    r = Ring()
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ""
    frame_ms = 1000 // FPS
    next_frame = time.ticks_ms()
    # quick start-up sweep so you can see the ring is alive
    for i in range(NUM_LEDS * 2):
        r.handle("N %d 1" % (i % NUM_LEDS))
        r.step(time.ticks_ms())
        r.render(time.ticks_ms())
        time.sleep_ms(20)
    while True:
        while poll.poll(0):
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                if buf:
                    reply = r.handle(buf)
                    if reply:
                        print(reply)
                    buf = ""
            elif len(buf) < 64:
                buf += ch
        now = time.ticks_ms()
        if time.ticks_diff(now, next_frame) >= 0:
            r.step(now)
            r.render(now)
            next_frame = time.ticks_add(next_frame, frame_ms)
            if time.ticks_diff(now, next_frame) > 0:
                next_frame = now          # fell behind; don't try to catch up
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
