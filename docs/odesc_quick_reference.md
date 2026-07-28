# ODESC V4.2 — Quick Reference & Config Record

Beetle's four wheel drive motors. Written 2026-07-22 after bringing all four boards from bare to calibrated.

---

## The setup in one paragraph

Four Flipsky ODESC V4.2 boards, one per hub motor, running ODrive firmware 0.5.6 on hardware 3.6-56V. Each drives a 24 V hoverboard hub motor and reads an AS5047P magnetic encoder over SPI for closed-loop control. EGO 56 V pack → **24 V DC regulator** → all four drivers. Each board has its own 2 Ω 50 W brake resistor on a shared fan-cooled heatsink. Encoder position comes back to the X5 over CAN as ODrive Encoder Estimates.

---

## Environment quirks — read before debugging anything

These cost hours to rediscover.

| Quirk | Detail |
|---|---|
| **`dev0` not `odrv0`** | odrivetool 0.5.4 names non-genuine boards `devN`. Every command in this doc uses `dev`. |
| **"Not a genuine ODrive!"** on connect | Expected on Flipsky clones. Harmless. |
| **odrivetool version must match firmware** | Use **0.5.4** (0.5.6 isn't on PyPI). A newer odrivetool breaks `dump_errors`, hides the firmware version in the banner, and leaves enum constants undefined. |
| **Enum constants undefined** | `GPIO_MODE_DIGITAL` etc. raise `NameError`. Use integers, or `import odrive.enums as e`. |
| **`save_configuration()` returns `False` silently** | The axis wasn't idle. Set `requested_state = 1` first, confirm `current_state == 1`, save again. |
| **Saves auto-reboot on 0.5.4** | Connection drops and reconnects. Save one board at a time and wait. |
| **`axis1` reports `MOTOR_ERROR_DRV_FAULT`** | Standing condition on the unused axis of every board. Harmless for single-motor use. Only worth attention if it appears on axis0. |
| **`identify` doesn't exist** | LED-blink board identification is 0.6.x only. Use motor calibration (state 4) as an audible identifier, or spin the motor. |
| **No `commutation_encoder`/`load_encoder` split** | That's the 0.6.x / ODrive Pro API. One encoder per axis here — the AS5047P does both commutation and position. Halls are unused. |
| **CAN properties live under `axis.config.can`** | It's `axis0.config.can.node_id` and `axis0.config.can.encoder_rate_ms`, not `can_node_id` / `encoder_msg_rate_ms`. |
| **`dc_max_negative_current` has a history of reverting** | Check it specifically after every save. |
| **Errors latch** | A cleared fault still reads until `clear_errors()`. Always clear before concluding a fix failed. |

---

## Board inventory

| | Serial (decimal) | CAN node | Wheel position | R (Ω) | L (µH) | SPI err rate | Encoder dir |
|---|---|---|---|---|---|---|---|
| dev0 | 62041683472693 | 0 | right_front | 0.2652 | 337 | 0.0036 | 1 |
| dev1 | 58807586992437 | 1 | left_rear | 0.2587 | 339 | 3.9e-07 | 1 |
| dev2 | ~~58803294777653~~ **63063866028595** | 2 | right_rear | 0.2506 | 338 | 0.0010 | 1 |
| dev3 | 62041683145013 | 3 | left_front | 0.2652 | 360 | 0.0042 | 1 |

Phase resistance and inductance are within ~3% across all four — good cross-check
that the phase harnesses are wired consistently.

**2026-07-28 — dev2 replaced.** A USB cable worked loose, wrapped a motor and
destroyed the original right_rear board (58803294777653). Replacement is
**63063866028595** / hex `395B33553233`, configured with `setup(d, 2)`.

Calibrated and measured: 0.2506 Ohm / 338 uH, SPI error rate 0.0010, encoder
direction 1, encoder error 0. Inductance is mid-pack; resistance is the lowest
of the four, widening the spread from 2.6% to 5.8%. Within what shunt tolerance
and crimp variation produce, but noted here so that if this wheel ever behaves
oddly under load, it is on record as always having been the low-resistance one.

Route that corner's USB harness deliberately when remounting. A loose cable
wrapping the motor is what destroyed the previous board.

Serial appears in `odesc_teleop_node.py` and `odesc_tuner_node.py` — both
updated. It is duplicated between them, so a future swap means editing two
files; worth extracting to a shared module at some point.

To get serials as hex (matching the connection banner, for physical labels):

```python
for i, d in enumerate([dev0, dev1, dev2, dev3]):
    print(i, format(d.serial_number, 'X'), d.axis0.config.can.node_id)
```

**Wheel positions assigned** (audited 2026-07-22, table above).
Still outstanding: Label each board physically with its hex serial and node ID, and photograph the mounted layout with labels visible before they're hard to reach.

---

## Working config (all four boards, as of 2026-07-22)

```python
def setup(d, node_id):
    d.axis0.requested_state = 1                              # IDLE, required for saves

    # Power / bus
    d.config.brake_resistance = 2.0                          # per-board resistor
    d.config.enable_brake_resistor = True
    d.config.dc_max_negative_current = -5.0                  # regen allowance
    d.config.dc_max_positive_current = 4.5                   # 4 boards x 4.5 = 18A on a 20A regulator
    # d.config.max_regen_current stays 0 — regulator can't sink

    # Motor
    d.axis0.motor.config.pole_pairs = 15
    d.axis0.motor.config.motor_type = 0                      # HIGH_CURRENT
    d.axis0.motor.config.resistance_calib_max_voltage = 4
    d.axis0.motor.config.calibration_current = 5
    d.axis0.motor.config.current_lim = 20                    # phase current
    d.axis0.motor.config.current_control_bandwidth = 100

    # Encoder
    d.axis0.encoder.config.mode = 257                        # SPI_ABS_AMS
    d.axis0.encoder.config.abs_spi_cs_gpio_pin = 6
    d.axis0.encoder.config.cpr = 16384                       # 14-bit

    # Controller
    d.axis0.controller.config.control_mode = 2               # VELOCITY_CONTROL
    d.axis0.controller.config.vel_gain = 0.02
    d.axis0.controller.config.vel_integrator_gain = 0.1
    d.axis0.controller.config.vel_limit = 5
    d.axis0.controller.config.vel_limit_tolerance = 2.5

    # CAN
    d.axis0.config.can.node_id = node_id
    d.axis0.config.can.encoder_rate_ms = 10                  # 100 Hz

    d.save_configuration()
```

Run one board at a time — each save reboots and drops the connection.

### setup() is the starting point, NOT the running configuration

**Learned the hard way on 2026-07-28.** After `setup()` and calibration, a
replacement board still disagreed with the other three on six parameters,
because those were tuned live through the Deck tuner and saved, and the tuned
values were never written down. `setup()` gets a board to a safe, working
baseline; these are what the robot actually runs on:

| Parameter | setup() | Running value |
|---|---|---|
| `axis0.controller.config.vel_gain` | 0.02 | **2.43** |
| `axis0.controller.config.vel_integrator_gain` | 0.1 | **0.0130451** |
| `axis0.controller.config.vel_limit` | 5 | **5.5** |
| `axis0.motor.config.current_lim` | 20 | **22** |
| `axis0.motor.config.torque_constant` | *(not written)* | **0.128901** |
| `config.dc_max_negative_current` | -5.0 | **-5.5** |

Total loop gain between the two columns differs by roughly 390x — `vel_gain`
alone is 121x, with `torque_constant` 3.2x on top. A board on setup() defaults
is not merely untuned, it is limp, and the difference is invisible until the
wheel is loaded.

`torque_constant` is the one to watch: `setup()` does not write it at all, so a
new board sits on the ODrive default of 0.04.

**Replacing a board, in order:**

1. `setup(d, node_id)` and full calibration (sequence above)
2. `odesc_compare.py` -> `compare([dev0, dev1, dev2, dev3])` to see what differs
3. `sync_from(good_board, new_board, apply=True)` to copy exactly, one board only
4. Re-run `compare()` to confirm
5. Update the serial in `odesc_teleop_node.py` AND `odesc_tuner_node.py`
6. Verify direction wheels-up before the floor

Do not restore a config blob: it would carry the source board's `node_id`
across, and copying an encoder offset from a different motor/encoder/magnet
assembly produces a wheel that takes off instead of holding position.

If these values are ever retuned, update the table above. It is the only
record, and rediscovering it costs an evening.

### Why these values

- **`pole_pairs = 15`** — hoverboard hub motors, near-universal.
- **`current_control_bandwidth = 100`** — the default 1000 rad/s is tuned for low-inductance motors. At ~0.25 Ω / ~340 µH the electrical time constant is ~1.4 ms, and 1000 causes `MOTOR_ERROR_CONTROL_DEADLINE_MISSED`.
- **`resistance_calib_max_voltage = 4`** — default 2.0 often fails calibration on these motors with `ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`.
- **`dc_max_negative_current = -5.0`** — default is `-0.01`, which vetoes essentially all regen. Any deceleration trips `ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT` and drops to idle. **This was the single most confusing fault in bringup.**
- **`dc_max_positive_current = 4.5`** — sized so four boards at limit stay under the 20 A regulator.
- **`vel_limit_tolerance = 2.5`** — multiplier on `vel_limit` before overspeed trips. Keeps transient overshoot from faulting during tuning.
- **`encoder.config.mode = 257`** — `ENCODER_MODE_SPI_ABS_AMS`, for AS5047P/AS5048A.
- **`abs_spi_cs_gpio_pin = 6`** — Flipsky puts CS on IO6 (stock ODrive convention is 7). Read the silkscreen.

### Not yet correct

`torque_constant` is **wrong on all four** — 0.5169 on dev1 (from a guessed 16 KV), 0.04 default on the others. Formula is `8.27 / KV`.

Doesn't affect velocity or position control. It matters for:
- Torque mode (planned for pushing/towing)
- Meaningful current-to-torque numbers in the energy model for the locomotion policy

**Plan:** mount a spare motor in the lathe for a constant known RPM, measure back-EMF. `KV = RPM / (V_ac_line_to_line × 1.732)`. Measure several of the same model to see batch spread, and log raw RPM/voltage rather than just derived KV. ~26 spare motors plus other models available — worth building a KV + phase-resistance table across the stock.

---

## Wiring

### AS5047P → ODESC SPI header

Six pins on the header: GND, MOSI, MISO, SCK, CS-IO6, 3.3V

| AS5047P board | ODESC header |
|---|---|
| 3V/5V — **jumper on 3.3V** | 3.3V |
| GND | GND |
| CSn | CS-IO6 |
| CLK | SCK |
| MOSI | MOSI |
| MISO | MISO |

Leave unconnected: A, B, I/PWM, W/PWM, V, UP1, TEST, NC

- Jumper **must** be 3.3V — the ODESC's SPI lines are 3.3V logic and the STM32 pins aren't 5V-tolerant.
- `UP1` is a programming/OTP pin. Don't touch it.
- `TEST` stays low or floating.
- **ABI fallback:** A/B/I are available if SPI proves too noisy over long cable runs. `ENCODER_MODE_INCREMENTAL`, `cpr = 4096`, index search on startup. Loses absolute-on-boot. Not needed so far.

### Power topology

```
EGO 56V pack → 24V DC regulator (20A) → 4× ODESC
                                         each with its own 2Ω 50W brake resistor
                                         all resistors on a shared fanned heatsink
```

- **The regulator can't sink current.** All regen goes to the brake resistors; `max_regen_current` stays 0.
- At 24 V, peak into 2 Ω is ~290 W — well within a 50 W average rating with forced air for short decelerations.
- **The 60 V ODESC headroom concern does not apply here** — that was for a direct 58.8 V pack connection. At 24 V there's ample margin.
- **Future:** upgrading to a bidirectional regulator for true regenerative braking. Note the EGO BMS has no active disconnect, so it won't defend against overcharge either — charge-side protection has to live in the robot's electronics, same as discharge-side.

---

## Bringup sequence (new board, from bare)

1. Verify firmware: `dev0.fw_version_major, dev0.fw_version_minor, dev0.fw_version_revision` → want `(0, 5, 6)`. `(0, 0, 0)` means factory Flipsky image, needs flashing.
2. Wire the encoder, check jumper is 3.3V, confirm magnet is mounted and centered.
3. Run `setup(dev, node_id)`.
4. Check `dev.axis0.encoder.error` → want 0.
5. Encoder function test (below).
6. Motor calibration: `dev.axis0.requested_state = 4`
7. Encoder offset calibration: `dev.axis0.requested_state = 7` — **motor spins**
8. Persist: `pre_calibrated = True` on both motor and encoder configs, then idle, then save.
9. Closed-loop test, then regen test.
10. Verify persistence after reboot.

Full step-by-step with pass/fail criteria is in `odesc_encoder_bringup.md`.

---

## Encoder function test

The test that catches a bad joint or marginal magnet before anything is sealed up.

```python
import time
for i in range(20):
    print(dev0.axis0.encoder.count_in_cpr)
    time.sleep(0.2)
```

Run three times: stationary, turning slowly, turning fast.

| | Pass |
|---|---|
| Stationary | stable within ±10 counts of 16384 (±0.2°) |
| Slow turn | smooth monotonic, no jumps or dead spots |
| Fast turn | clean wrap at the boundary (e.g. 1909 → 15841) |

Direction doesn't matter — counting up or down are both fine; offset calibration resolves it against the phases.

### SPI error rate thresholds

```python
dev0.axis0.encoder.spi_error_rate
```

| Value | Meaning |
|---|---|
| < 1e-4 | clean |
| 1e-4 – 1e-3 | fine |
| 1e-3 – 1e-2 | working, but not much margin left |
| > 1e-2 | close to the trip point, expect intermittent faults |
| ~0.05 | ODrive faults with `ERROR_ABS_SPI_COM_FAIL` |
| ~1.0 | nothing coming back at all |

It's a **decaying running average** — it climbs during fault cycling and takes minutes to settle. Re-read after a few clean minutes before treating a number as real.

**Known open item:** dev1 sits at 3.9e-07 while dev0/dev2/dev3 sit around 0.004 — a ~10,000x spread across supposedly identical builds, and the three higher ones are at steady state, not still settling. dev1 was built first on a different cable; the other three were soldered in one session. All are far under the fault threshold, so this isn't blocking, but the three have ~12x margin where dev1 has ~100,000x. Worth revisiting when the cables reach full length at the wheels, since motor noise and vibration are still to come. The definitive test is swapping dev1's cable onto one of the others.

**SPI run length is the known design risk** — hub motors put the encoder at the wheel with a long cable back. Keep runs short, shield if possible. CRC errors surface on the ODESC as **commutation faults**, not as a clean "bad encoder" message, which is why the bench baselines above are worth having.

---

## Everyday commands

### Connect and inspect

```python
# odrivetool launches and auto-connects
dev0.axis0.current_state          # 1 = IDLE, 8 = CLOSED_LOOP
dev0.vbus_voltage
dev0.axis0.encoder.pos_estimate   # turns, cumulative
dev0.axis0.encoder.vel_estimate   # turns/sec
dev0.axis0.motor.current_control.Iq_measured
```

### Drive a wheel

```python
dev0.axis0.requested_state = 8              # CLOSED_LOOP_CONTROL
dev0.axis0.controller.input_vel = 0.5       # turns/sec — stay under vel_limit (5)
dev0.axis0.controller.input_vel = 0
dev0.axis0.requested_state = 1              # back to IDLE
```

At `input_vel = 0` in closed loop the motor should hold position and resist being turned by hand. If it takes off instead, the encoder direction is wrong — kill it and re-run offset calibration.

### Compare all four

```python
for i, d in enumerate([dev0, dev1, dev2, dev3]):
    print(i, d.axis0.current_state, d.axis0.encoder.spi_error_rate,
          d.axis0.config.can.node_id, d.config.dc_max_negative_current)
```

### Dump one board's motor config

```python
for k in dir(dev0.axis0.motor.config):
    if not k.startswith('_'):
        print(k, getattr(dev0.axis0.motor.config, k))
```

### Errors

```python
from odrive.utils import dump_errors
dump_errors(dev0)
dev0.clear_errors()
```

Manual decode if `dump_errors` misbehaves:

```python
import odrive.enums as e

def decode(val, prefix):
    for k, v in e.__dict__.items():
        if k.startswith(prefix) and isinstance(v, int) and v and (val & v):
            print(hex(v), k)

decode(dev0.axis0.error, "AXIS_ERROR_")
decode(dev0.axis0.motor.error, "MOTOR_ERROR_")
decode(dev0.axis0.encoder.error, "ENCODER_ERROR_")
decode(dev0.axis0.controller.error, "CONTROLLER_ERROR_")
```

---

## Error codes seen during bringup

| Code | Name | Cause | Fix |
|---|---|---|---|
| `0x80` | `ENCODER_ERROR_ABS_SPI_COM_FAIL` | SPI wiring, jumper on 5V, MOSI/MISO swap, no/far magnet, unconfigured encoder mode | Check `encoder.config.mode == 257` first — an unconfigured board fails regardless of wiring |
| `0x40` | `AXIS_ERROR_MOTOR_FAILED` | downstream | read `motor.error` |
| `0x200` | `AXIS_ERROR_CONTROLLER_FAILED` | downstream | read `controller.error` |
| `0x1000000` | `MOTOR_ERROR_CONTROL_DEADLINE_MISSED` | `current_control_bandwidth` too high for the motor's L/R | set to 100 |
| — | `ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT` | `dc_max_negative_current` at its `-0.01` default | set to -5.0 |
| — | `MOTOR_ERROR_SYSTEM_LEVEL` | downstream of a system-level error | read the `system:` line in `dump_errors` |
| — | `MOTOR_ERROR_DRV_FAULT` on axis1 | unpopulated axis on a clone | ignore |

**Debugging order that actually worked:** check `encoder.config.mode` is 257 → clear latched errors → read `spi_error_rate` and `count_in_cpr` to see whether anything is coming back at all → only then suspect wiring.

---

## CAN

Not yet brought up — currently on USB. Bus is wired but not functional.

- All four boards have distinct node IDs (0–3) and `encoder_rate_ms = 10` (100 Hz).
- ODrive arbitration: `(node_id << 5) | cmd_id`. Encoder Estimates is `cmd_id 0x09`.
- Encoder Estimates payload: position in **turns as float32** + velocity in turns/s.
- Position is **cumulative turns, not absolute angle** — zeroed at power-up. Difference it for odometry; `mod 1.0` for absolute wheel angle. **Do not zero mid-run** — that injects a discontinuity into the signal the odometry integrator is differencing.
- **Bring up the four ODESCs alone first, then add the steering Pico.** Don't add a second protocol family to a bus that doesn't work yet.
- The steering Pico's node IDs must sit well outside 0–3.

---

## Safety notes

- **Torque mode has no speed regulation.** This is the mode that produced a ~5 second full-speed runaway (bad hall velocity sign, brake-on-release experiment). Torque control is wanted for pushing/towing, but the **deadman button should exist first** — it doesn't depend on any velocity reading, so it can't fail the way that experiment did.
- Every closed-loop entry on the bench: motor free-spinning, hand near the kill path.
- `vel_limit` is a real safety parameter, not just a tuning knob. Commanding above it trips overspeed and drops to idle — which is the desired behavior, so don't raise it casually.
- Calibration state 7 **spins the motor**. Clear the area.

---

## Open items

- [ ] Assign and physically label wheel positions per board (serial + node ID); photograph mounted layout
- [ ] Measure real KV on the lathe, set `torque_constant` on all four
- [ ] Investigate the dev1-vs-others SPI error rate spread (swap dev1's cable onto another board)
- [ ] Re-baseline SPI error rates after cables reach final length at the wheels
- [ ] Confirm calibration survives a full power cycle (enter closed loop without recalibrating)
- [ ] CAN bringup — ODESCs alone first
- [ ] Deadman button before any torque-mode work
- [ ] Update `01_robot_design.md` — the power section still records EGO pack → ODESCs direct, with the 60 V headroom problem and brake resistors as a hard requirement. Actual build has a 24 V regulator in between, which changes that analysis.
- [ ] Bidirectional regulator for regenerative braking (future)
