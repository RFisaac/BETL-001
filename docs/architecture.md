# Architecture — as built, Stage 3

This describes what actually runs on Beetle as of 2026-07-28. It supersedes the
architecture described in [`PROPOSAL.md`](../PROPOSAL.md) (Stage 2), which was written
before the robot existed and got several concrete decisions wrong in the normal way a
pre-build proposal does. Where the two disagree, this document is current.

**If a doc anywhere in this repo disagrees with the code, the code is right.** That
happened enough times during Stage 3 bring-up (see [`failure-recovery.md`](failure-recovery.md))
that it's worth saying explicitly.

---

## Platform

Skid-steer robot: four hoverboard hub motors, one Flipsky ODESC V4.2 driver each, RDK X5
compute, Intel RealSense D435i (RGB + IMU), and an RPLIDAR A1 canted 90° on an
encoder-driven rotating stage — a single 2D lidar reprojected into a 3D "poor-man's
spherical scanner." Teleop from a Steam Deck over UDP, or a browser console over
rosbridge. Skid steer stayed skid steer for the competition build; a swerve-drive redesign
exists as a separate, later design thread (see the build journal) but never reached this
robot.

## What's real vs. what was tried and pulled

**Depth is off. RTAB-Map is not in the live chain.** Enabling RealSense depth
(`align_depth`) pushed the board to 111% CPU and a 16.4 load average, which broke
everything downstream of it. RTAB-Map existed to consume that depth stream for visual
odometry; it's gone from the running system. Its CPU cost under concurrent load is still
real, useful data — it's what "concurrent" means in the benchmark (see
[`benchmark.md`](benchmark.md)) — but it is not doing SLAM on the robot today.

**This is dead reckoning, not SLAM.** Pose comes from wheel encoders (forward velocity)
fused with gyro yaw rate (rotation), integrated forward in time. There is **no loop
closure and no global correction** — good for minutes of gentle driving, not hours, and it
will drift. Calling this "SLAM" anywhere in submission material would be inaccurate; the
concurrent-BPU-vs-CPU-load claim is supported by the RTAB-Map benchmark numbers
separately, without needing RTAB-Map to be live.

## Node graph

```
wheel encoders (ODESC, USB) ─┐
                              ├─► wheel_odom_node ──► /wheel_odom, TF odom → camera_link
RealSense gyro (BEST_EFFORT) ┘      (forward v from wheels, yaw rate from gyro)

camera_link ──(static)──► stage_base ──(scan_stage_node, 150 Hz)──► stage_mount ──(static, 90° cant)──► laser

/scan (RPLIDAR A1, ~6.5 Hz)
   └─► scan_accumulator_node ──► /cloud_3d  (per-segment TF deskew, voxelised, self-filtered)
                              └─► hud_view_node ──► /hud/map  (z-sliced, thinned, for the browser console)

RealSense RGB ──► dnn_node_example (BPU, YOLOv8n INT8) ──► /beetle_detections
```

**Full TF chain:** `odom → camera_link → stage_base → stage_mount → laser`. A single TF
lookup at each point's own timestamp corrects for both the robot's motion and the stage's
rotation at once — that's the whole trick behind the spherical reprojection; the base and
stage motion are not deskewed separately.

**The gate that actually matters:** `odom → laser` must not just exist, it must *change on
its own* while the stage turns. A frozen-but-present transform passes every naive check and
silently produces a useless, static cloud. `nodes/beetle_up.py` checks this explicitly
(`TFMoving`, not just `TFExists`) because it's the failure mode a fixed-delay launch file
can't catch.

## The two concurrent workloads (Stage 3 Challenge 2)

- **BPU:** YOLOv8n INT8, 640×640, fed from the RealSense RGB stream, via
  `dnn_node_example`. Detection ROIs come back in model space (640×640) and must be scaled
  per-axis into image space (640×480) — they land off-frame otherwise.
- **CPU:** wheel+gyro dead reckoning (`wheel_odom_node`), lidar reprojection and voxel
  accumulation (`scan_accumulator_node`), the console/telemetry stack, and — as the
  concurrent-load stand-in for a full SLAM pipeline — RTAB-Map's CPU cost, captured
  separately (see `benchmark.md`).

These run on physically separate compute units (BPU vs. A55 cores), which is why BPU
inference held essentially flat under concurrent CPU load in the benchmark while every
CPU-side stage degraded — they're contending for memory bandwidth, not for the same
compute.

## Bring-up: gated, not timed

`nodes/beetle_up.py` starts each stage and **verifies** it before moving to the next,
rather than waiting a fixed number of seconds and hoping (`nodes/beetle_bringup.launch.py`
is the fast, ungated alternative for when the hardware is already known-good). Every gate
is a machine-checkable claim — a topic publishing at a minimum rate, a TF existing, a TF
*moving*, a port listening — and a failure prints the offending node's log tail and asks
`[r]etry [s]kip [a]bort` instead of silently continuing against a dead dependency. Full
gate list and the bringup sequence: [`baseline_run.md`](baseline_run.md).

## Always-on: the agent

`nodes/beetle_agent.py` runs from boot via systemd (`systemd/beetle-agent.service`) and
needs no ROS. It does three things: broadcasts a UDP discovery beacon so the Deck can find
the robot on any network, serves the browser console (`console/beetle_console.html`) on
port 8080, and launches/supervises `beetle_up.py` on request, streaming its gate output to
the browser. It exists specifically so a failed bring-up is *visible* — the console speaks
rosbridge, which doesn't exist until the stack is up, so something outside ROS has to be
the thing that gets you there and shows you what happened.

## Safety — single path to the motors

Every velocity source funnels through the drive node (`nodes/odesc_teleop_node.py`), which
owns the only path to the four ODESCs. Layered, in order:

1. **Deck deadman** (upstream) — release zeros `/cmd_vel`.
2. **Command watchdog** on the drive node — no `/cmd_vel` for `cmd_timeout` (0.3 s) → zeros.
3. **`/estop` topic** (`std_msgs/Bool`) — `true` latches: zeros the drives and drops to
   IDLE. Publish `false` to clear and re-enter closed loop.
4. **ODrive firmware watchdog** — armed at runtime, fed every control tick; if the node or
   the USB link dies, each axis faults to idle on its own, independent of software.
5. **Wheel-speed clamp** (`max_wheel_tps`, run at 0.6 t/s ≈ 0.29 m/s — well under
   `vel_limit=5`), applied **proportionally across both sides**, never per-side (per-side
   clamping saturates both sides to the same value at full throttle and the robot can no
   longer steer).
6. **Hardware e-stop** — physical, outside software, always available.

**Velocity control only.** Torque mode is not used — it's the mode that produced a ~5 s
full-speed runaway during an earlier brake-on-release experiment (bad hall velocity sign),
and it stays off until a hardware deadman-on-release exists independent of any velocity
reading. A partial drivetrain (fewer than four wheels reaching closed loop) is refused by
default rather than driven — a three-wheel skid-steer robot veers, it doesn't degrade
gracefully, so `allow_partial` exists only for stationary bench work.

## Calibration and known issues

Measured constants (wheel radius, track width, gyro bias) and how they were obtained:
[`calibration.md`](calibration.md). Bugs found during bring-up with transferable lessons,
plus the open accumulator-overlay issue: [`failure-recovery.md`](failure-recovery.md).
