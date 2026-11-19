# Code index

Robot-side code (`nodes/`, `ros2_ws/`, `systemd/`) runs on the RDK X5 from `/userdata`. Operator-side code (`tools/`, `console/`)
runs on the Steam Deck or a laptop. Everything here is the **Stage 3 (skid-steer) version**. The mecanum changes run on the robot
but haven't been pushed yet.

**Status:** 🟢 in daily use · 🟡 works, parked (Stage 3 stack, no current use) · 🔁 needs a mecanum update

## Robot side — `nodes/`

| File | What it does | Status |
|---|---|---|
| [`udp_cmdvel_node.py`](udp_cmdvel_node.py) | Deck UDP packets → `/cmd_vel` Twist | 🟢 🔁 (needs vy) |
| [`odesc_teleop_node.py`](odesc_teleop_node.py) | `/cmd_vel` + `/estop` → four ODESCs over USB, with watchdogs and clamp | 🟢 🔁 (skid-steer mix → mecanum) |
| [`odesc_tuner_node.py`](odesc_tuner_node.py) | Drive node + live ODESC parameter tuning for `deck_tuner.py` | 🟢 🔁 |
| [`odesc_compare.py`](odesc_compare.py) | Read-only diff of all four ODESC configs | 🟢 |
| [`beetle_agent.py`](beetle_agent.py) | Always-on (systemd): discovery beacon, browser console, supervised bringup | 🟢 |
| [`beetle_up.py`](beetle_up.py) | Gated bringup that verifies each stage before starting the next | 🟡 |
| [`beetle_bringup.launch.py`](beetle_bringup.launch.py) | Fast, ungated launch of the full stack | 🟡 |
| [`wheel_odom_node.py`](wheel_odom_node.py) | Wheel + gyro dead reckoning → `odom → camera_link` | 🟡 🔁 (mecanum kinematics) |
| [`scan_accumulator_node.py`](scan_accumulator_node.py) | Spinning-lidar `/scan` → per-point TF deskew → `/cloud_3d` | 🟡 |
| [`hud_view_node.py`](hud_view_node.py) | Thins `/cloud_3d` into something a browser can draw | 🟡 |
| [`drive_log.py`](drive_log.py) | Time-aligned CSV of wheel states + odometry | 🟡 |
| [`yolo_bench.py`](yolo_bench.py), [`capture_benchmark.sh`](capture_benchmark.sh) | BPU benchmark harvesting and evidence capture | 🟡 |
| [`../ros2_ws/src/scan_stage/`](../ros2_ws/src/scan_stage/) | Lidar stage angle → TF (colcon package) | 🟡 |
| [`../systemd/beetle-agent.service`](../systemd/beetle-agent.service) | Starts the agent at boot | 🟢 |

## Operator side — `tools/` and `console/`

| File | What it does | Status |
|---|---|---|
| [`../tools/deck_teleop.py`](../tools/deck_teleop.py) | Steam Deck controller → UDP, with deadman | 🟢 🔁 (crab-lock, speed trim not pushed) |
| [`../tools/deck_tuner.py`](../tools/deck_tuner.py) | Drive and tune ODESC parameters from the Deck | 🟢 |
| [`../tools/beetle_deck.sh`](../tools/beetle_deck.sh) | One command on the Deck: teleop + console | 🟢 |
| [`../tools/beetle_find.py`](../tools/beetle_find.py) | Find the robot on any network | 🟢 |
| [`../console/beetle_hud.html`](../console/beetle_hud.html) | The amber CRT HUD | 🟢 |
| [`../console/beetle_console.html`](../console/beetle_console.html) | Browser telemetry and tuning console (rosbridge) | 🟡 |

How it all fits together: [`../docs/architecture.md`](../docs/architecture.md) · deploy order: [`../docs/deploy.md`](../docs/deploy.md)
