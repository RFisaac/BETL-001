# Beetle — Deployment Checklist

Everything from the 2026-07-28 session. Work top to bottom; the order matters
in two places, both called out.

> **Path note:** written when everything sat flat in one working folder. This repo now
> splits those files into `nodes/`, `console/`, `tools/`, and `systemd/` (see the root
> [`README.md`](../README.md#repository-layout)) — the filenames below are unchanged, just
> read them as `nodes/wheel_odom_node.py`, `console/beetle_console.html`,
> `systemd/beetle-agent.service`, etc. relative to the repo root. `scan_stage_node.py` is
> the one exception; it lives in [`ros2_ws/src/scan_stage/`](../ros2_ws/src/scan_stage/)
> because it's the one node that's an actual installed colcon package (§2 below).

---

## 1. Robot — `/userdata/`

Plain scripts, run directly. Copy from the laptop:

```powershell
scp wheel_odom_node.py odesc_tuner_node.py odesc_teleop_node.py `
    scan_accumulator_node.py drive_log.py `
    beetle_up.py beetle_agent.py beetle_bringup.launch.py `
    root@192.168.86.58:/userdata/
```

| File | What changed |
|---|---|
| `wheel_odom_node.py` | IMU QoS fix (BEST_EFFORT), `gyro_scale`, `learn_bias`, `min_wheels_moving=2`, r=0.0765, track=0.375 |
| `odesc_tuner_node.py` | `max_wheel_tps` now exists **and is live**, `~/clear_errors` service, r/track |
| `odesc_teleop_node.py` | `publish_position` (off by default), r/track |
| `scan_accumulator_node.py` | `pose x= y= z= yaw=` on the health line |
| `drive_log.py` | wheel + odometry CSV logger |
| `beetle_up.py` | gated bringup supervisor |
| `beetle_agent.py` | boot agent: beacon + web + launcher |
| `beetle_bringup.launch.py` | optional fast path, no gates |

`beetle_beacon.py` is superseded by `beetle_agent.py`. Don't copy it.

---

## 2. Robot — `scan_stage_node.py` is DIFFERENT

**This one is not in `/userdata`.** It's an installed ROS package, running from
`/root/beetle_ws/install/scan_stage/...`, so copying it to `/userdata` does
nothing — `ros2 run` will keep using the old build.

```bash
find /root/beetle_ws/src -name scan_stage_node.py     # locate the source
# copy the new file over that path, then:
cd /root/beetle_ws && colcon build --packages-select scan_stage
source ~/beetle_ws/install/setup.bash
```

**Verify it took** — without this the console's stage-zero knob has nothing to
talk to:

```bash
ros2 run scan_stage scan_stage_node --ros-args -p angle_offset_deg:=0.0
# starts cleanly = new build. Parameter error = still the old one.
```

---

## 3. Robot — web root `/root/beetle_hud/`

```bash
mkdir -p /root/beetle_hud
# from the laptop:
#   scp beetle_console.html root@192.168.86.58:/root/beetle_hud/
ls /root/beetle_hud/roslib.min.js     # must already exist from the HUD setup
```

If `roslib.min.js` is missing, fetch it **while the board still has internet**:

```bash
cd /root/beetle_hud && curl -LO https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js
```

That matters before you move to the portable router — the console is dead
without it and there's no CDN out there.

---

## 4. Robot — output directories

```bash
mkdir -p /userdata/beetle/{logs,clouds,bench,bags,renders}
```

`logs/` is new — `beetle_up.py` writes one file per node there.

---

## 5. Robot — dependency check

```bash
ros2 pkg list | grep -E "rosbridge|foxglove|rplidar|realsense|scan_stage"
```

`rosbridge_server` is the one to confirm — every console knob is a service
call through it, and nothing else in the stack needs it, so it's the most
likely gap.

**If it's missing:** `apt install ros-humble-rosbridge-suite`. Install that
one package only. **Never `apt upgrade`** on this board — 500+ pending
updates including BPU packages.

---

## 6. Test bringup BY HAND before enabling anything at boot

Don't skip this ordering. A systemd service that starts a broken bringup is
much harder to debug than a broken bringup.

```bash
python3 /userdata/beetle_up.py --drive-only
```

Expect green `ok` on: camera IMU ≥100 Hz, camera_info 640×480, `/wheel_states`
4 wheels, `/wheel_odom` ≥10 Hz, TF `odom → camera_link`.

Then the full stack:

```bash
python3 /userdata/beetle_up.py --spin --console
```

The gate to watch is **`TF odom -> laser is MOVING`**. That's the one that's
been failing invisibly.

Failures prompt `[r]etry [s]kip [a]bort` and print the node's log tail.

---

## 7. Robot — agent at boot

Only after step 6 passes:

```bash
cp /userdata/beetle-agent.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now beetle-agent
systemctl status beetle-agent
journalctl -u beetle-agent -f
```

Startup output should list each interface with its IP and MAC, then the
beacon and console lines.

**Check for a port clash:** if anything from an earlier session is already on
8080, the agent won't bind.

```bash
ss -tlnp | grep 8080
```

---

## 8. Deck

```
beetle_find.py          → anywhere on PATH, or next to deck_teleop.py
deck_teleop.py          → already there; unchanged
beetle_console.html     → NOT needed here, the robot serves it
```

Test discovery with the robot powered up:

```bash
python3 beetle_find.py -v        # should print the IP
python3 beetle_find.py --pin     # remember this robot's MAC
python3 beetle_find.py --url     # console URL to open in the browser
```

Optional one-line change to `deck_teleop.py` so `--host` becomes optional:

```python
ap.add_argument("--host", default=None, help="X5 IP, or omit to auto-discover")
...
from beetle_find import find_robot
host = args.host or find_robot() or sys.exit("robot not found")
```

## 9. Laptop

`beetle_find.py` is useful here too — the ARP fallback handles both Linux and
Windows `arp -a` output.

---

## Full sequence, once installed

1. Power on the X5. Agent starts by itself.
2. Deck: `python3 beetle_find.py --url`, open it in the browser.
3. Console shows **ROBOT OFFLINE**. Pick options, press **START ROBOT**.
4. Watch the gates stream in. Amber is passing, red is not.
5. Overlay clears when rosbridge connects.
6. Deck: `python3 deck_teleop.py --host $(python3 beetle_find.py)` and drive.

---

## Before the first real run

- [ ] `timedatectl` — clock synced. An NTP correction mid-run scrambles
      stamped TF lookups and looks exactly like an odometry failure.
- [ ] `ls -l /dev/rplidar` — udev symlink resolves. It pointed at `ttyUSB1`
      on the 27th and `ttyUSB0` on the 28th; that's why the rule exists.
- [ ] Foxglove, if used: fixed frame **`odom`**, display frame **`odom`**,
      follow mode **Fixed**. `map` no longer exists — it was RTAB-Map's.
- [ ] Console node names match `ros2 node list`. The `NODES` block at the top
      of `beetle_console.html` assumes `/scan_stage_node`, `/wheel_odom`,
      `/odesc_tuner_node`, `/scan_accumulator`.

---

## Still open

**The accumulator overlay.** Rooms built on top of each other with a fixed
origin. Wheel-radius scale is ruled out — odometry was over-reporting by 7.8%,
which can't turn 3 m into 1 m. The `pose` field on the accumulator health line
is the instrument; the two-blob test in `baseline_run.md` is the procedure.

**Rooms not square after turning.** Heading scale. Use the console's yaw
calibration: MARK, turn one full revolution, APPLY 360°.
