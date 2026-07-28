# Calibration — measured constants

Everything below was measured on 2026-07-28 and is baked into the node defaults in
`nodes/`. If a number here disagrees with an older doc (including `PROPOSAL.md`), this
page wins — it's the last thing actually measured on the hardware.

| Quantity | Value | How measured |
|---|---|---|
| Wheel radius (effective, used for odometry) | **0.0765 m** | 3.5 m taped straight-line run |
| Wheel diameter (calipers, geometric) | 155.0–155.5 mm → r = 0.0776 m | Four wheels, ±0.25 mm |
| Slip | **1.4%** | Difference between the taped-run radius and the caliper radius; hard rubber, smooth floor, gentle throttle |
| Track width | **0.375 m** | Direct measurement |
| Gyro bias | ~0.0008 rad/s (0.046°/s) | Stationary, zero-velocity-update convergence |
| Gyro scale | 1.0003 → **left at 1.0** | 359.9° measured over one commanded revolution |
| Max speed at the 0.6 t/s wheel-speed cap | 0.288 m/s | `0.6 × 2π × 0.0765` |

**The wheel radius was previously recorded as 0.0825 m** — an eyeballed tape measurement
that was never actually a spec, and it over-reported every distance by 7.8%. If you're
reading an older doc or an earlier commit that uses 0.0825 m, it's wrong; 0.0765 m is the
measured value.

## Gyro bias and why it dominates heading error

A stationary gyro never reads exactly zero, and integrating that offset is pure heading
drift. `wheel_odom_node.py` tracks bias continuously using the wheels as a zero-motion
reference (classic zero-velocity update, ZUPT): whenever the wheels report stopped, the raw
gyro reading *is* the bias, and it gets averaged in.

Gyro **scale** turned out not to matter much in practice — predicted 1–3% error, measured
0.03%. **Bias dominates heading error by roughly 150×**: over a 3-minute run with 180° of
total turning, scale contributed about 0.05° of error and bias contributed about 8.3°. Any
future heading-accuracy work should target bias hygiene, not scale.

**Before a run:** let the robot sit parked for 20–30 s so bias converges (gyro warm-up
drift settles on that timescale), and watch the `GYRO BIAS` readout on the console's
ODOMETRY panel. Healthy is ~0.0008 rad/s; anything above 0.01 is a poisoned estimate, not
just a drifty sensor — call the `reset_bias` service (or hit RESET BIAS in the console) and
let it re-converge rather than trusting a run that started from a bad bias.

**`learn_bias` must be set `false`** on any fixture where the robot rotates with the wheels
stopped (a turntable, a hand-push) — otherwise the zero-velocity update averages that real
rotation into the bias estimate as if it were noise.

**Below `min_wheels_moving` (2 wheels) the robot is treated as parked** and yaw is held,
regardless of single-wheel encoder noise; above it, rotation is tracked as real. This
replaced an earlier all-or-nothing parked guard that was gated on `learn_bias` alone, which
meant a turntable or a hand-nudge only registered as motion during active calibration.

## Wheel odometry vs. gyro yaw — why the split

Wheels measure forward distance well and rotation badly: skid-steer scrubs sideways
through every turn, so differential wheel speed over-reports yaw by an unknown,
load-and-surface-dependent amount. A gyro is the opposite — excellent short-term rotation,
useless for position. `wheel_odom_node.py` takes forward velocity from the wheels and yaw
rate from the gyro, which is why swerve (eliminating scrub entirely) would make wheel
rotation a genuine odometry source rather than something to be overridden by the gyro — but
that's future work, not this build.

## ODESC drive tuning

The four Flipsky ODESC V4.2 drivers each carry six parameters that were live-tuned through
the Deck tuner and never matched their `setup()` defaults — a board replaced mid-competition
(`right_rear`, 2026-07-28) came up on defaults and was, in the literal sense, undriveable
under load until synced. Full config record, wiring, error codes, and the sync procedure:
[`odesc_quick_reference.md`](odesc_quick_reference.md).

The short version: **`setup()` is a starting point, not the running configuration.** Loop
gain differs by roughly 390× between a board on `setup()` defaults and a board running the
tuned values — a board is not merely "untuned," it is limp, and that's invisible until the
wheel is actually loaded. If a board is ever replaced again, run
`nodes/odesc_compare.py`'s `compare()` before assuming a fresh `setup()` is good enough.
