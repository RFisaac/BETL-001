# Failure modes & recovery

Safe-shutdown and e-stop behavior is in [`architecture.md`](architecture.md#safety--single-path-to-the-motors);
this page covers what fails, how it's detected, and how it recovers or is worked around.

## Automatic recovery paths

| Failure | Detection | Recovery |
|---|---|---|
| Deck link drops mid-drive | Command watchdog, 0.3 s timeout | `/cmd_vel` zeros automatically |
| Drive node process dies / USB link dies | ODrive firmware watchdog, fed every control tick | Each ODESC axis faults to idle independently of the X5 |
| One ODESC drops out mid-run | Per-wheel state polled at 5 Hz; anything not `CLOSED_LOOP` while not e-stopped | Latching stop — a 3-wheel skid-steer robot veers rather than degrades, so all four go to IDLE. Clear with `/estop true` then `false` |
| RealSense IMU drops out | `wheel_odom` health line: `yaw_src` flips from `GYRO` to `wheels` | Odometry silently falls back to wheels-only (scrub-corrupted) heading — **this is a degraded state, not a crash**; watch for it, don't assume it can't happen |
| RPLIDAR enumerates on a different `ttyUSBn` | N/A — this is prevented, not detected | udev rule pins it to `/dev/rplidar` (`/etc/udev/rules.d/99-beetle.rules`); never reference `ttyUSBn` directly |
| Stage timestamp wraps (every 17.9 min, MicroPython `ticks_us()` rolling at 2³⁰ µs) | Transforms stamped ~1074 s in the past, tf2 rejects as `TF_OLD_DATA` | Self-heals within `offset_reset_sec` (set to 5 s, down from the 30 s default, to shrink the damage window). The real fix — wrap-aware `ticks_diff` arithmetic on the Pico — is still outstanding |
| `beetle-agent` restarted via `systemctl restart` | N/A — this was a bug, now fixed | SIGTERM is not `KeyboardInterrupt`; signal handlers now call `bringup.stop()` explicitly so a restart doesn't orphan a full stack and start a second one alongside it |

## Manual recovery / operator procedures

- **Encoder SPI fault** (long cable run from wheel-mounted encoder back to the driver): shows up on the ODESC as a *commutation fault*, not a clean "bad encoder" message. Check `spi_error_rate` per board (`odesc_quick_reference.md`) before assuming it's mechanical.
- **`degraded_tf` climbing on the accumulator health line**: stale TF, the cloud will be scrambled. Drop `segments` from 16 to 8 rather than raising `tf_timeout` — raising the timeout looks like a fix but blocks the callback thread and collapsed throughput from 5.9 to 0.9 scans/s in testing (an 86% scan loss, far worse than some stale stage angles).
- **Partial drivetrain** (fewer than 4 wheels reach closed loop at startup): refused by default (`allow_partial:=false`) rather than driven — set only for stationary bench work.

## Bugs found during Stage 3 bring-up, with transferable lessons

Almost every one of these presented as something else entirely — silent failures dominate
this list, which is the actual lesson underneath all of them: **prefer loud failures**
(refuse to start, latch a fault) **over silent degradation** wherever the two are a choice.

1. **IMU QoS mismatch.** RealSense publishes sensor data `BEST_EFFORT`; a `RELIABLE`
   subscriber does not match and receives *nothing*, with only a startup warning to show
   for it. `wheel_odom` had been silently running on scrub-corrupted wheels-only heading
   since it was written, with no error.
2. **Per-side speed clamp destroyed steering.** Clamping left and right independently
   means both saturate to the same value at full throttle, so the robot can't turn. Fixed
   by scaling both sides by one common factor.
3. **`odesc_tuner_node` never declared `max_wheel_tps`.** ROS 2 accepts undeclared
   parameters on the command line without error, so `-p max_wheel_tps:=0.6` was silently
   ignored for weeks. Every scan collected before 2026-07-28 was at an unknown, unclamped
   speed.
4. **`gyro_axis:=y` is boolean `true` in YAML.** `y`, `yes`, `on` are all true in YAML 1.1,
   which `rcl`'s parameter parser follows — it arrives as `True` against a string
   parameter and throws before the node starts. Quote it: `-p "gyro_axis:='y'"`.
5. **rosbridge glob syntax.** `[*]` fails `yaml.safe_load` (`*` is a YAML alias marker);
   `['*']` yields a `STRING_ARRAY` the node rejects. The correct form is `'[*]'` — quotes
   *inside* the value, so YAML resolves it to the plain string `[*]`.
6. **Stage timestamps wrap every 17.9 minutes** — see the automatic-recovery table above.
   Root cause understood; the real fix (wrap-aware arithmetic) is still outstanding.
7. **Agent orphaned processes on `systemctl restart`.** SIGTERM ≠ `KeyboardInterrupt`, so
   cleanup never ran, leaving a full stack running; the next start launched a second one
   alongside it — two YOLO nodes were observed publishing to the same topic. Fixed with
   explicit signal handlers.
8. **Detection ROIs are in model space (640×640), not image space (640×480).** Boxes must
   be scaled per-axis or they land off-frame.
9. **Two executors on one node.** `TransformListener(spin_thread=True)` adds the node to
   its own background-thread executor; also calling `spin_once` on that node drops
   callbacks. Presented as a static transform that was demonstrably being published and
   yet could not be looked up.
10. **USB enumeration order is not stable.** The lidar was `ttyUSB1` one day and `ttyUSB0`
    the next depending on what else was plugged in. Fixed with a udev symlink.
11. **A ROS node's name is not its filename.** `odesc_tuner_node.py` declares itself
    `odesc_tuner`. Every console service call aimed at `/odesc_tuner_node` failed silently —
    a service call to a node that doesn't exist simply times out — so three console
    features looked like dead buttons rather than a naming bug. **Always check `ros2 node
    list`, never assume the filename.** The other four nodes happened to match their
    filenames and hid the pattern for a while.

## Open issue — the accumulator overlay

**Status: unresolved, highest-value open item.** Driving into another room sometimes
produces a cloud built *on top of* the original room, with the origin apparently fixed and
a rotated copy of the first room overlaid on itself.

Ruled out:
- **Scale.** Odometry was over-reporting distance by 7.8% (the wheel-radius bug above);
  that error cannot turn 3 m of real travel into 1 m of apparent travel.
- **The integrator.** `odom_path / dist_wheels = 1.0000` over a 4.56 m run, checked twice.
- **Foxglove display settings.** They can make a stationary origin *look* static, but
  cannot collapse two spatially separated point sets into one.

**Instrumented, not yet diagnosed:** `scan_accumulator_node` logs the pose it actually
applies on its health line (`pose x=+0.16 y=+3.25 z=+0.00 yaw=-36.9`). Diagnostic procedure
is the two-blob test in [`baseline_run.md`](baseline_run.md#the-two-blob-test--do-this-before-the-baseline-run):
clear, sit still 10 s, drive 2–3 m, sit still 10 s. Two separated dense blobs means it's
working; one blob with a moving `pose x=` means the pose is right and the accumulated
points aren't using it — a bug in the accumulator, not in odometry.

## Smaller known items, not currently blocking

- `WHEELS` (serial → wheel-position mapping) is duplicated in `odesc_teleop_node.py` and
  `odesc_tuner_node.py`. A board swap means editing both files by hand; nothing enforces
  agreement between them.
- `beetle_up.py`'s drive-readiness gate checks that four wheels *report*, not that they're
  fault-free — a board latched in `ENCODER_FAILED` currently passes the gate and only fails
  on the first real drive command.
- SPI error rates sit around 0.005 on three ODESC boards with one consistently near zero;
  which board is the low one has changed twice with no cable touched. Under the fault
  threshold, so not currently a problem, but worth revisiting once cable runs reach their
  final length at the wheels.
