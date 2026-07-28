#!/usr/bin/env python3
"""
odesc_compare.py — read-only diff of all four ODESC boards.

    # On the X5 (Python 3.10) — stop the drive node first, it holds the USB:
    python3 /userdata/odesc_compare.py

    # From odrivetool, which is the reliable route on Windows / Python 3.13:
    #   odrivetool          # let it connect dev0..dev3
    #   exec(open('odesc_compare.py').read())
    #   compare([dev0, dev1, dev2, dev3])

WHY THE ODRIVETOOL ROUTE EXISTS
  odrive 0.5.4's find_any() has a bug:

      current_timeout = min(0, timeout - (time.monotonic() - wait_start))

  min, not max — so the value is always 0, wait(0) returns instantly, and the
  loop spins without ever timing out. On Python 3.10 it yields enough for USB
  discovery to complete anyway; on 3.13 the spin starves the discovery thread
  and it hangs forever. odrivetool reaches discovery by another path, so
  handing this its already-connected devices sidesteps the bug entirely.

Writes NOTHING. Safe to run any time, and the right first move after
replacing a board: it shows exactly which settings the new one disagrees on,
so you change those deliberately instead of broadcasting a whole config and
hoping.

WHY NOT JUST RESTORE A CONFIG BLOB
  Some of these are per-board by nature. node_id must differ. The calibration
  results (phase resistance, inductance, encoder offset) belong to one
  specific board, motor, encoder and magnet — copying them across is how you
  get a wheel that takes off instead of holding position at zero velocity.
  This tool separates the two: settings you WANT identical, and results you
  expect to differ.
"""

import sys
import time

import odrive

# position -> serial (decimal). Keep in step with WHEELS in the drive nodes.
BOARDS = {
    "left_front":  "62041683145013",
    "left_rear":   "58807586992437",
    "right_front": "62041683472693",
    "right_rear":  "63063866028595",   # replaced 2026-07-28
}

# Settings that SHOULD match across all four. Everything setup() writes, plus
# the parameters odesc_tuner_node exposes for live tuning.
SHOULD_MATCH = [
    ("config.brake_resistance",                      "brake resistance"),
    ("config.enable_brake_resistor",                 "brake resistor on"),
    ("config.dc_max_positive_current",               "dc max +I"),
    ("config.dc_max_negative_current",               "dc max -I"),
    ("axis0.motor.config.pole_pairs",                "pole pairs"),
    ("axis0.motor.config.motor_type",                "motor type"),
    ("axis0.motor.config.resistance_calib_max_voltage", "R calib max V"),
    ("axis0.motor.config.calibration_current",       "calib current"),
    ("axis0.motor.config.current_lim",               "current limit"),
    ("axis0.motor.config.current_control_bandwidth", "I ctrl bandwidth"),
    ("axis0.motor.config.torque_constant",           "torque constant"),
    ("axis0.encoder.config.mode",                    "encoder mode"),
    ("axis0.encoder.config.abs_spi_cs_gpio_pin",     "encoder CS pin"),
    ("axis0.encoder.config.cpr",                     "encoder cpr"),
    ("axis0.controller.config.control_mode",         "control mode"),
    ("axis0.controller.config.vel_gain",             "vel gain"),
    ("axis0.controller.config.vel_integrator_gain",  "vel integrator"),
    ("axis0.controller.config.vel_limit",            "vel limit"),
    ("axis0.controller.config.vel_limit_tolerance",  "vel limit tol"),
    ("axis0.config.can.encoder_rate_ms",             "CAN encoder rate"),
    ("axis0.motor.config.pre_calibrated",            "motor pre-calibrated"),
    ("axis0.encoder.config.pre_calibrated",          "encoder pre-calibrated"),
]

# Expected to differ, or specific to one board. Shown for reference only.
PER_BOARD = [
    ("serial_number",                          "serial"),
    ("axis0.config.can.node_id",               "node id"),
    ("axis0.motor.config.phase_resistance",    "phase R (ohm)"),
    ("axis0.motor.config.phase_inductance",    "phase L (H)"),
    ("axis0.encoder.config.offset",            "encoder offset"),
    ("axis0.encoder.config.offset_float",      "encoder offset f"),
    ("axis0.encoder.config.direction",         "encoder dir"),
    ("axis0.encoder.error",                    "encoder error"),
    ("axis0.motor.error",                      "motor error"),
    ("axis0.error",                            "axis error"),
]


def get_path(obj, path):
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


def fmt(v):
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, int) and v > 0xFFFF:
        return str(v)
    return str(v)


def compare(devices):
    """Compare already-connected devices.

    Accepts a list (order irrelevant — boards are matched to wheel positions
    by serial number) or a {position: device} dict."""
    if isinstance(devices, dict):
        devs = dict(devices)
    else:
        by_serial = {str(s): p for p, s in BOARDS.items()}
        devs = {}
        for d in devices:
            pos = by_serial.get(str(d.serial_number))
            if pos is None:
                print(f"serial {d.serial_number} "
                      f"({format(d.serial_number,'X')}) is not in BOARDS "
                      f"— skipping")
                continue
            devs[pos] = d
    missing = [p for p in BOARDS if p not in devs]
    if missing:
        print(f"not connected: {', '.join(missing)}\n")
    if len(devs) < 2:
        print("need at least two boards to compare")
        return 1
    order = [p for p in BOARDS if p in devs]
    w = 22

    print("\n" + "=" * 78)
    print("SHOULD MATCH — anything flagged is a real difference to reconcile")
    print("=" * 78)
    print(f"{'setting':<{w}}" + "".join(f"{p[:13]:>15}" for p in order))
    n_diff = 0
    for path, label in SHOULD_MATCH:
        vals, shown = [], []
        for p in order:
            try:
                v = get_path(devs[p], path)
                vals.append(v)
                shown.append(fmt(v))
            except Exception:
                vals.append(None)
                shown.append("--")
        good = [v for v in vals if v is not None]
        differ = len(set(map(str, good))) > 1
        n_diff += differ
        line = f"{label:<{w}}" + "".join(f"{s:>15}" for s in shown)
        print(line + ("   <-- DIFFERS" if differ else ""))

    print("\n" + "=" * 78)
    print("PER BOARD — differences here are expected, not faults")
    print("=" * 78)
    print(f"{'property':<{w}}" + "".join(f"{p[:13]:>15}" for p in order))
    for path, label in PER_BOARD:
        shown = []
        for p in order:
            try:
                shown.append(fmt(get_path(devs[p], path)))
            except Exception:
                shown.append("--")
        print(f"{label:<{w}}" + "".join(f"{s:>15}" for s in shown))

    print("\n" + "=" * 78)
    if n_diff:
        print(f"{n_diff} setting(s) differ.")
        print("Set each from the Deck tuner — it writes to ALL FOUR boards —")
        print("then send `save` to commit. Confirm the value you are pushing")
        print("is the one from a KNOWN GOOD board, not the new one.")
    else:
        print("All shared settings agree.")
    print("Phase R/L should sit within a few percent across boards; wider")
    print("than that suggests a phase harness wired differently.")
    return 0


def sync_from(good, target, apply=False):
    """Copy the SHOULD_MATCH settings from one board to another.

    Only touches `target`. Per-board values — serial, node_id, calibration
    results — are never copied, which is the whole reason this exists rather
    than a config restore.

        import odesc_compare as oc
        oc.sync_from(dev0, dev3)              # dry run, prints the diff
        oc.sync_from(dev0, dev3, apply=True)  # write, then save

    save_configuration() reboots the board, so the odrivetool handle for it
    goes stale afterwards — reconnect before using it again."""
    changes = []
    for path, label in SHOULD_MATCH:
        try:
            src = get_path(good, path)
            dst = get_path(target, path)
        except Exception:
            continue
        if str(src) == str(dst):
            continue
        changes.append((path, label, dst, src))

    if not changes:
        print("nothing to copy — already identical")
        return 0

    print(f"{'setting':<22}{'target now':>16}{'will become':>16}")
    for path, label, dst, src in changes:
        print(f"{label:<22}{fmt(dst):>16}{fmt(src):>16}")

    if not apply:
        print(f"\n{len(changes)} change(s). Dry run — call again with "
              f"apply=True to write.")
        return 0

    # Config writes need the axis idle, and so does save_configuration().
    target.axis0.requested_state = 1
    time.sleep(0.3)
    if target.axis0.current_state != 1:
        print(f"axis is in state {target.axis0.current_state}, not IDLE — "
              f"aborting rather than writing into a live axis")
        return 1

    failed = []
    for path, label, dst, src in changes:
        try:
            set_path(target, path, src)
        except Exception as e:
            failed.append(f"{label}: {e}")
    for f in failed:
        print("  FAILED " + f)

    print("saving (the board reboots, this handle goes stale)...")
    try:
        target.save_configuration()
    except Exception:
        pass          # the save reboots mid-request; losing the reply is normal
    print("done. Reconnect and re-run compare() to confirm.")
    return 0


def set_path(obj, path, value):
    parts = path.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


def watch_spi(devices, seconds=30, period=2.0):
    """Sample encoder SPI error rates over time.

    A single reading means little: spi_error_rate is an exponentially-weighted
    moving average over control cycles, so it tracks RECENT conditions rather
    than being a fixed property of a harness. Between 2026-07-22 and 07-28 the
    quietest board changed identity twice with no cable touched, which is what
    prompted this.

    What the shape tells you:
      steady across all four      shared environment, not per-cable
      one board climbing          that harness — reseat, check the 3.3V jumper
      all climb when motors spin  EMI from the phase leads coupling into SPI
      spiky, uncorrelated         marginal connector seating

    Run it idle first, then again with the wheels turning; the difference is
    the interesting part."""
    if isinstance(devices, dict):
        devs = dict(devices)
    else:
        by_serial = {str(v): k for k, v in BOARDS.items()}
        devs = {by_serial.get(str(d.serial_number), str(d.serial_number)): d
                for d in devices}
    order = list(devs)
    print(f"{'t':>6}" + "".join(f"{p[:13]:>15}" for p in order))
    hist = {p: [] for p in order}
    t0 = time.time()
    while time.time() - t0 < seconds:
        row = []
        for p in order:
            try:
                v = float(devs[p].axis0.encoder.spi_error_rate)
            except Exception:
                v = float("nan")
            hist[p].append(v)
            row.append(f"{v:>15.3e}")
        print(f"{time.time()-t0:6.0f}" + "".join(row))
        time.sleep(period)

    print("\n" + "-" * (6 + 15*len(order)))
    for label, fn in (("min", min), ("max", max)):
        print(f"{label:>6}" + "".join(
            f"{fn([x for x in hist[p] if x == x] or [0]):>15.3e}" for p in order))
    print(f"{'trend':>6}" + "".join(
        f"{(hist[p][-1] - hist[p][0]):>+15.3e}" for p in order))
    print("\ntrend is last minus first — positive means it is getting worse "
          "while you watch.")
    return hist


def main():
    devs = {}
    for pos, serial in BOARDS.items():
        hexser = format(int(serial), "X")
        print(f"connecting {pos} ({hexser})...", flush=True)
        try:
            d = odrive.find_any(serial_number=hexser, timeout=15)
        except Exception as e:
            print(f"  FAILED: {e}")
            continue
        if d is None:                 # 0.5.4 returns None rather than raising
            print("  not found")
            continue
        devs[pos] = d
    if not devs:
        print("\nNothing connected. If this hung rather than failing, it is the"
              "\nfind_any() bug described at the top — use the odrivetool route.")
        return 1
    return compare(devs)


if __name__ == "__main__":
    sys.exit(main())
