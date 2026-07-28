# Beetle — Bringup & Baseline Run

Current architecture: **no depth, no RTAB-Map in the live chain.** Pose comes from
`wheel_odom_node` (wheels + gyro). Supersedes `clean_bringup.md`, which still
describes the depth/RTAB-Map path.

**Every step has a gate. Do not proceed past a failed check.** Almost every
confusing symptom in this project has turned out to be a node started before its
dependency was actually alive.

Board: `root@192.168.86.58` · both `source` lines are already in `~/.bashrc`

**TF chain this builds:**

```
odom ──► camera_link ──► stage_base ──► stage_mount ──► laser
   wheel_odom      static        scan_stage       static
     20 Hz                        150 Hz
```

**Measured constants** (2026-07-28 — all defaults in the nodes, don't pass them):

| | Value | Source |
|---|---|---|
| Wheel radius (effective) | **0.0765 m** | 3.5 m tape run; 1.4% slip below the 0.0776 m caliper geometric radius |
| Wheel diameter (calipers) | 155.0–155.5 mm | four wheels, ±0.25 mm |
| Track width | **0.375 m** | measured |
| Gyro bias | ~0.0008 rad/s (0.05°/s) | stable across a 4-minute run |
| Slip | 1.4% | hard rubber, smooth floor, gentle throttle |

**Known: the stage timestamp wraps every 17.9 minutes.** MicroPython's
`time.ticks_us()` rolls at 2³⁰ µs = 1073.7 s, and `scan_stage_node` subtracts
raw tick values instead of using `ticks_diff`. At each wrap, transforms are
stamped ~1074 s in the past, tf2 rejects them as `TF_OLD_DATA`, and the stage
link goes stale until `offset_reset_sec` (30 s) recomputes the offset. It
self-heals. Time runs to avoid it, or shorten the damage window:

```bash
ros2 run scan_stage scan_stage_node --ros-args \
  -p lidar_frame:=stage_mount -p offset_reset_sec:=5.0
```

---

## 0. Clean slate

```bash
tmux kill-server
```

That drops SSH. Reconnect, then sweep:

```bash
ssh root@192.168.86.58

pkill -f realsense2_camera
pkill -f static_transform_publisher
pkill -f wheel_odom
pkill -f rplidar
pkill -f scan_stage
pkill -f scan_accumulator
pkill -f odesc
pkill -f dnn_node_example
pkill -f detection_overlay
pkill -f foxglove
sleep 3
```

**Gate** — must return nothing but a daemon:

```bash
ros2 node list
```

Still listed → `pkill -9 -f <name>`.

```bash
tmux new -s beetle
```

New window `Ctrl+b` `c` · rename `Ctrl+b` `,` · switch `Ctrl+b` `w`

---

## 1. Window `camera` — RealSense, RGB + IMU only

**No depth.** Depth plus `align_depth` is what drove load average to 16.4 and
killed everything downstream. This config runs ~11% CPU.

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  enable_depth:=false enable_infra:=false align_depth.enable:=false \
  rgb_camera.color_profile:=640x480x15
```

Wait ~10 s for enumeration.

**Gate:**

```bash
ros2 topic hz /camera/camera/imu     # ~200 Hz
ros2 topic hz /camera/camera/color/image_raw    # ~15 Hz
```

If IMU isn't flowing, nothing downstream works — `wheel_odom` needs it for yaw.

---

## 2. Window `drive` — teleop

Must come **before** `wheel_odom`, which subscribes to `/wheel_states`.

```bash
cd /userdata
python3 odesc_tuner_node.py --ros-args -p max_wheel_tps:=0.6
```

`0.6` t/s ≈ 0.29 m/s at the measured 0.0765 m radius. **Do not leave this at the
default 2.0** — 0.96 m/s smears the lidar badly enough that a bad cloud looks
like an odometry problem.

**This clamp only started working on 2026-07-28.** `odesc_tuner_node.py` never
declared `max_wheel_tps` and never applied it, and ROS 2 accepts undeclared
parameters on the command line without error — so every earlier tuner run was
unclamped regardless of what was passed. Wheels were measured at 0.689 t/s
against a supposed 0.6 limit. Any scan data from before that date was collected
at unknown speed.

Falls back to Path B (`udp_cmdvel_node.py` + `odesc_teleop_node.py`, two windows)
if the tuner misbehaves. Same speed cap either way.

**Gate** — all four wheels present, velocities near zero:

```bash
ros2 topic echo /wheel_states --once
```

---

## 3. Window `odom` — wheel + gyro odometry

```bash
python3 /userdata/wheel_odom_node.py
```

Every value you'd otherwise pass is already the default (`use_imu_yaw` true,
`gyro_axis` `"y"`, `gyro_sign` −1), so the bare command is the safe form.

**Do not write `-p gyro_axis:=y`.** Bare `y` is boolean true in YAML 1.1, which
rcl's parameter parser follows, so it arrives as `True` against a string
parameter and throws `InvalidParameterTypeException` before the node starts. If
you want it explicit, it has to be quoted:

```bash
python3 /userdata/wheel_odom_node.py --ros-args \
  -p use_imu_yaw:=true -p "gyro_axis:='y'" -p gyro_sign:=-1
```

**Gate — the QoS line.** If startup prints:

```
New publisher discovered on topic '/camera/camera/imu', offering
incompatible QoS. No messages will be received from it.
```

the node is receiving zero IMU data and has silently fallen back to wheels-only
yaw. Confirm the fix landed: the health line must read `yaw_src=GYRO` and `n=`
must climb. `n=0` with `yaw_src=wheels` means the patched file didn't make it to
the board.

**Leave the robot parked and untouched from here on.** Bias is learned while the
wheels read stopped, and anything that rotates the robot now gets averaged into
it.

**Gate — bias convergence.** `n=` reaches 200 within a couple of seconds, but keep
watching: compare `bias=` across two consecutive health lines, 5 s apart. It
should be stable in the fourth decimal. Give it ~30 s so gyro warm-up drift
settles. Also confirm `yaw_src=GYRO`, not `wheels`.

Bias is the single number the whole run's heading accuracy rests on. This is the
cheapest place to buy accuracy — spend the 30 seconds.

**Gate:**

```bash
ros2 run tf2_ros tf2_echo odom camera_link
```

Must stream, values holding near zero while parked.

Start the next steps while bias settles — they don't disturb the robot.

---

## 4. Windows `tf1` and `tf2` — static transforms

**Own window each, no trailing `&`.** Backgrounded ones get orphaned and vanish.

`tf1` — mounting offset (and the stage-zero yaw knob):

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -0.117 --y 0.057 --z 0.038 --roll 0 --pitch 0 --yaw 0 \
  --frame-id camera_link --child-frame-id stage_base
```

`tf2` — the 90° lidar cant:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.0505 --roll 0 --pitch 1.5708 --yaw 0 \
  --frame-id stage_mount --child-frame-id laser
```

**Gate:**

```bash
ros2 node list | grep -c static_transform_publisher    # must be 2
```

---

## 5. Window `lidar` — RPLIDAR A1

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar -p serial_baudrate:=115200 -p frame_id:=laser
```

**Use `/dev/rplidar`, never `ttyUSBn`.** Enumeration order is not stable — the
lidar was `ttyUSB1` on 2026-07-27 (a second CP210x device was plugged in) and
`ttyUSB0` on 2026-07-28 after it was removed. Pointing the driver at the wrong
node opens the port fine and then times out at the handshake, which looks like a
dead lidar. The udev rule in `/etc/udev/rules.d/99-beetle.rules` pins the name:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

If `ls -l /dev/rplidar` shows nothing, the rule didn't load —
`udevadm control --reload && udevadm trigger`.

**Gate:**

```bash
ros2 topic hz /scan     # ~6.5 Hz
```

---

## 6. Window `stage` — rotating stage

```bash
ros2 run scan_stage scan_stage_node --ros-args -p lidar_frame:=stage_mount
```

**Gate:**

```bash
ros2 topic hz /stage_angle     # ~150 Hz
```

Start it turning:

```bash
ros2 topic pub --once /stage/set_duty std_msgs/msg/Int32 "{data: 40000}"
```

Duty floor is **36000**; below that it stalls. Stop with `{data: 0}` or
`ros2 service call /stage/stop std_srvs/srv/Trigger`.

---

## 7. THE GATE — full chain

```bash
ros2 run tf2_ros tf2_echo odom laser
```

Must stream **and** the rotation must change on its own with the stage turning.

This one check covers `odom → camera_link → stage_base → stage_mount → laser`.
**If it fails, do not start the accumulator** — it will drop every scan.

"Not part of the same tree" → the break is `odom → camera_link`, so step 3.
Everything below `camera_link` is static or the stage.

---

## 8. Window `cloud` — accumulator

```bash
cd /userdata/beetle/clouds
python3 /userdata/scan_accumulator_node.py --ros-args \
  -p fixed_frame:=odom -p base_frame:=camera_link \
  -p mask_frame:=camera_link \
  -p mask_min_x:=-0.717 -p mask_max_x:=0.133 \
  -p mask_min_y:=-0.243 -p mask_max_y:=0.357 \
  -p mask_min_z:=-0.962 -p mask_max_z:=0.088 \
  -p segments:=8 -p tf_timeout:=0.05 \
  -p voxel_size:=0.05 -p publish_hz:=1.0 -p max_voxels:=200000
```

**Why `mask_frame:=camera_link` and the shifted box.** The accumulator only
reuses the base transform when `mask_frame == base_frame`; otherwise it does a
second stamped lookup per segment. Matching them halves the TF lookups (16 → 8
per scan at `segments:=8`, down from 32 at the default 16). The box bounds above
are the original `stage_base` box shifted by the `camera_link → stage_base`
translation `(-0.117, 0.057, 0.038)`, which is exact **only while that static
publisher's `--yaw` is 0**. When stage-zero calibration sets a nonzero yaw, revert
`mask_frame` to `stage_base` and the original bounds.

**Do not raise `tf_timeout`.** It looks like it fixes `degraded_tf` — it does —
but `lookup_transform` blocks the calling thread for up to that duration, and at
0.15 s throughput collapsed from 5.9 to 0.9 scans/s against a 6.42 Hz lidar.
Losing 86% of scans is far worse than some stale stage angles.

**Gate** — the 5 s health line shows `scans=` climbing and `points=` growing:

```
scans=266 points=41288 dropped=1 masked=8609 degraded_tf=39 | pose x=+0.16 y=+3.25 z=+0.00 yaw=-36.9
```

**The `pose` field is the pose actually being applied** to incoming points. It is
the direct answer to "is the cloud respecting odometry" — watch it while driving:

- `x`/`y` walking, matching `wheel_odom` → pose is being applied correctly
- `x`/`y` frozen while `yaw` moves → rotation applied, translation not
- both frozen while `wheel_odom` reports motion → stale or cached transform

Also confirm **scans/s**, not just that `scans` climbs. Against a 6.42 Hz lidar,
4+ per second is healthy; under 2 means scans are being dropped by DDS before
`on_scan` ever sees them, which is a CPU problem, not a settings problem.

Check the first line's `masked=` against `points=`. The self-filter box is still
untuned, so if it's eating most of the cloud, add `-p mask_enable:=false` and
re-run — a too-large box removing real wall geometry would ruin the measurement.

- `dropped` climbing with `scans=0` → step 7 wasn't really passing
- `degraded_tf` climbing → stale TF, cloud will be scrambled; drop `segments` to 8
- `points` pinned at 200000 → voxel cap, raise `max_voxels`

Add `-p publish_body:=true` if you also want the body-frame cloud for tuning the
mask box later. Costs a little CPU; skip it on the first run.

---

## 9. Window `bag` — record the run

Do this. Re-driving is expensive; re-processing is free.

```bash
cd /userdata/beetle/bags
ros2 bag record -o baseline_$(date +%H%M) \
  /scan /wheel_states /camera/camera/imu /wheel_odom \
  /tf /tf_static /stage_angle
```

**No camera images** — raw RGB would be ~14 MB/s and swamp everything. These
topics together are a few MB per minute.

What this buys: re-running the accumulator with different `voxel_size`,
`segments`, or mask settings against the identical drive, and having the raw
gyro and wheel data for offline analysis once the calibration numbers exist.

---

## 10. Optional — Foxglove, YOLO, overlay

Not needed for the measurement. Foxglove is, if you want to watch the cloud build
live rather than reading the `.pcd` afterward.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Connect to `ws://192.168.86.58:8765`.

**Panel settings — check these before interpreting anything:**

| Setting | Value | Why |
|---|---|---|
| Fixed frame | **`odom`** | `map` no longer exists — it was RTAB-Map's, and RTAB-Map is out of the live chain. A saved layout pointing at `map` is stale in exactly the way the old docs were. |
| Display frame | **`odom`** | Anything else and the world moves instead of the robot |
| Follow mode | **Fixed** | "Position" or "Pose" locks the camera to the robot, so it appears stationary no matter what odometry does |

3D panel, topic `/cloud_3d`, color by `z`.

**Foxglove settings cannot make two separated point sets overlap.** Point
coordinates are computed into `odom` and baked into the message before publishing,
so if two rooms render on top of each other, they overlap in the data. Display
settings can only explain an origin that appears not to move.

**Bandwidth:** `foxglove_bridge` subscribes on demand — cost is set by which
panels are open, not by what's published. An Image panel on
`/camera/camera/color/image_raw` is ~14 MB/s of rgb8 and will hold a full CPU
core; `/beetle_overlay` at 10 Hz is ~9 MB/s. `/cloud_3d` at 1 Hz is ~1 MB/s.
Close image panels for any run where scan throughput matters.

YOLO and the overlay, if you want them in the same recording:

```bash
cd /userdata
ros2 run dnn_node_example example --ros-args \
  -p config_file:=config/yolov8workconfig.json \
  -p feed_type:=1 -p is_shared_mem_sub:=0 \
  -p ros_img_topic_name:=/camera/camera/color/image_raw \
  -p image_width:=640 -p image_height:=480 \
  -p dump_render_img:=0 -p msg_pub_topic_name:=/beetle_detections \
  --log-level warn

python3 /userdata/detection_overlay.py --ros-args -p publish_hz:=10.0
```

---

## The two-blob test — do this BEFORE the baseline run

**Purpose:** confirm the cloud respects odometry at all. Until this passes, a
baseline run can't be interpreted — on 2026-07-27 a drive into the next room
produced a cloud built on top of the original room with the origin apparently
fixed, and the cause is still open.

Takes about a minute and needs no tape.

1. Foxglove frames set per section 10 — **`odom` / `odom` / Fixed**
2. Clear: `ros2 service call /scan_accumulator/clear std_srvs/srv/Trigger`
3. **Sit still 10 s** — builds a dense blob of room A
4. **Drive 2–3 m** in a straight line
5. **Sit still 10 s** — builds a dense blob of room B
6. Save: `ros2 service call /scan_accumulator/save std_srvs/srv/Trigger`

Watch the accumulator's `pose x=` field throughout.

| Result | Meaning |
|---|---|
| Two dense blobs, separated by the distance driven | Working. Proceed to the baseline run. |
| One blob; `pose x=` walked correctly | Pose is right but points aren't using it — accumulator bug |
| One blob; `pose x=` frozen while `wheel_odom` moved | Transform isn't reaching the accumulator |
| One blob; `pose` and `wheel_odom` both frozen | Odometry stopped — check `yaw_src=GYRO` |

The stationary pauses are what make this readable. Driving produces sparse smear;
standing still produces a dense recognizable blob, and comparing two blobs is far
easier than judging whether a smear moved.

Cross-check while it runs, in another window:

```bash
ros2 run tf2_ros tf2_echo odom camera_link
```

If `tf2_echo` shows translation while `pose x=` sits still, the problem is in the
accumulator's TF buffer, not in the transform.

---

## The baseline run

**Goal:** drive the same straight line twice, in opposite directions, so the same
wall gets scanned twice. The angle between the two copies *is* the heading error.
No ground truth, no tape measure — the geometry does the work.

**Pick the site:** a straight wall 4–6 m long with floor clear enough to drive
parallel to it, roughly 1–2 m out. A corridor is ideal. The wall needs to be the
most prominent flat thing in the scan.

**Note the wall-clock time you start and stop.** Bias error scales with elapsed
time and scale error doesn't, so the duration is what lets you separate them
later.

### Sequence

1. **Clear the cloud** immediately before starting:
   ```bash
   ros2 service call /scan_accumulator/clear std_srvs/srv/Trigger
   ```
2. **Mark the start.** Tape on the floor at a known chassis corner.
3. **Sit still 10 s.** Gives a clean stationary block at the head of the bag.
4. **Drive out** along the wall, slow and as straight as you can manage. Small
   steering corrections are fine — the gyro sees them.
5. **Stop, wait 5 s.** Clean break between legs, and the accumulator catches up.
6. **Turn 180°** in place.
7. **Drive back** along the same line to the start mark.
8. **Stop, wait 10 s.**
9. **Measure the return offset** — how far the chassis corner ended from the tape,
   along-track and lateral. Write it down.
10. **Save the cloud:**
    ```bash
    ros2 service call /scan_accumulator/save std_srvs/srv/Trigger
    ```
    Writes a timestamped `.pcd` to `/userdata/beetle/clouds/`. Ctrl-C also saves,
    but call the service — then the node keeps running for a second attempt.

Stop the bag with `Ctrl+c` in its window.

### While it runs, watch two windows

- **`cloud`** — `degraded_tf` must stay near zero. If it climbs, the cloud is
  scrambled for TF reasons and no odometry conclusion can be read through it.
  Abort and fix rather than finishing the run.
- **`odom`** — `yaw_src=GYRO` throughout. If it flips to `wheels`, the IMU
  dropped out and the yaw estimate is now scrub-corrupted.

---

## Reading the result

Load the `.pcd`, or look at `/cloud_3d` in Foxglove from directly above.

| What you see | What it means |
|---|---|
| Wall appears twice, at an angle | **Heading error** — the expected outcome |
| Wall doubled but parallel, offset sideways | Lateral drift from the turn, not scale |
| Return doesn't reach the start along the wall | Distance error — wheel radius or slip |
| Wall single but thick and fuzzy | Deskew or TF timing, not odometry |
| Slices scattered, no recognizable geometry | `degraded_tf` — TF problem, rerun |

**The angle is a gyro scale estimate.** Nearly all of the heading error comes from
the 180° turn: if true scale is $S$, odometry records $S \times 180°$ and the two
wall copies sit $(S-1) \times 180°$ apart. So

$$S \approx 1 + \frac{\text{angle between walls}}{180°}$$

A 5° gap means roughly 3% scale error. That's a real first-order number, and it
should agree with what the turntable produces later — if they disagree badly,
something more interesting is going on than scale.

The straight legs contribute bias × elapsed-time on top, which is why you noted
the duration. At an expected ~0.1°/s residual, a two-minute run adds a few
degrees, so the two effects are comparable in magnitude and the turntable run is
what separates them cleanly.

**If the walls land nearly on top of each other,** the defaults are better than
expected, and calibration is a refinement rather than a rescue. Also a fine
outcome — and worth knowing before spending an evening on fixtures.

---

## Failure quick-reference

| Symptom | Cause | Fix |
|---|---|---|
| `odom` frame doesn't exist | `wheel_odom` not publishing | Is `/wheel_states` flowing? Teleop must be up first |
| `yaw_src=wheels` | IMU dropped or timed out | Check `ros2 topic hz /camera/camera/imu` |
| `bias=` drifting, won't settle | Robot being bumped, or vibration nearby | Leave it alone 30 s |
| Accumulator `scans=0`, `dropped` climbing | Step 7 not actually passing | Fix TF first |
| `degraded_tf` climbing | Slow-link staleness | `segments` to 8 |
| Cloud flat / stacked in one plane | Foxglove display frame wrong | Display frame must be `odom` |
| Nodes vanish after a while | Started with `&` | Own tmux window each |
| Camera at 1280×720 | Wrong param name | `rgb_camera.color_profile`, not `.profile` |
| `RTPS_TRANSPORT_SHM Error` | Fast-RTPS shm contention | Harmless, falls back to UDP |
| Lidar `SL_RESULT_OPERATION_TIMEOUT` then segfault | Wrong `serial_port`, or nothing answering | Use `/dev/rplidar`. Segfault is downstream of the timeout — chase the timeout. `fuser -v` for a stale holder |
| `TF_OLD_DATA` for `stage_mount`, ~1074 s in the past | Pico `ticks_us()` wrapped at 2³⁰ µs | Self-heals in ≤ `offset_reset_sec`. Every 17.9 min |
| `InvalidParameterTypeException` on `gyro_axis` | Bare `y` is boolean true in YAML | Omit it (it's the default) or quote: `-p "gyro_axis:='y'"` |
| Health line reads `MOVING` while stationary | Single-wheel noise past the deadband | Fixed by `min_wheels_moving:=2`; was worth 0.2 m per run |
| Wheels exceed `max_wheel_tps` | Tuner never applied the clamp before 2026-07-28 | Update `odesc_tuner_node.py` |
| Scans/s far below `/scan` rate | Callback blocked, usually a raised `tf_timeout` | Return `tf_timeout` to 0.05, cut CPU load |
| `wheel_odom` gets no IMU, `n=0`, `yaw_src=wheels` | QoS mismatch — RealSense IMU is BEST_EFFORT | Fixed in the node; a startup QoS warning is the tell |
