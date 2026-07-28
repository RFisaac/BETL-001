#!/usr/bin/env python3
"""
beetle_up.py — gated bringup. Starts each stage, VERIFIES it, then continues.

    python3 /userdata/beetle_up.py                 # full stack
    python3 /userdata/beetle_up.py --drive-only    # camera + drives + odometry
    python3 /userdata/beetle_up.py --spin --console
    python3 /userdata/beetle_up.py --gyro-scale 1.023 --stage-zero -4.5

Ctrl-C shuts everything down in reverse order.

WHY THIS AND NOT THE LAUNCH FILE
  beetle_bringup.launch.py can only WAIT a fixed number of seconds. This
  checks. Every gate in baseline_run.md is really the question "is this
  actually publishing", which is machine-checkable — so a failure stops here,
  prints the offending node's last log lines, and asks, instead of silently
  starting the next thing against a dead dependency.

  The gate that matters most cannot be expressed in a launch file at all:
  odom -> laser must not merely EXIST, it must CHANGE on its own with the
  stage turning. A frozen-but-present transform looks completely healthy to
  every other check and produces a silent, useless cloud.

  Use the launch file when the hardware is known good and you want speed.
  Use this when you want to know.

LOGS
  Each node's output goes to /userdata/beetle/logs/<name>.log — this terminal
  stays readable. On a gate failure the tail of the relevant log is printed
  automatically, because that is where the real error always is.

    tail -f /userdata/beetle/logs/rtabmap.log      # watch one live
"""

import argparse
import json
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
import tf2_ros

from sensor_msgs.msg import Imu, JointState, LaserScan, CameraInfo
from nav_msgs.msg import Odometry

try:
    from ai_msgs.msg import PerceptionTargets
except ImportError:      # TROS not sourced, or ai_msgs absent
    PerceptionTargets = None

SCRIPTS = Path("/userdata")
CLOUDS = Path("/userdata/beetle/clouds")
LOGS = Path("/userdata/beetle/logs")

# Control channel. beetle_agent forwards console requests here so that ONE
# process owns every child — the agent cannot signal its grandchildren, and
# duplicating the command definitions on both sides is how they drift apart.
CONTROL_PORT = 5011

# Sensor topics publish BEST_EFFORT. A RELIABLE subscriber will NOT match a
# BEST_EFFORT publisher — DDS refuses the connection and delivers nothing,
# with only a QoS warning to show for it. That exact mismatch cost a night on
# /camera/camera/imu. BEST_EFFORT on our side matches both, always.
SENSOR_QOS = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# TransformListener(spin_thread=True) runs executor.spin() in a daemon
# thread. On Ctrl-C, rclpy tears the context down underneath it and that
# thread raises ExternalShutdownException — expected, harmless, and ugly.
# Filtered by name so a genuine thread crash still prints.
_orig_excepthook = threading.excepthook


def _quiet_shutdown(args):
    if args.exc_type.__name__ == "ExternalShutdownException":
        return
    _orig_excepthook(args)


threading.excepthook = _quiet_shutdown

C_OK, C_BAD, C_WARN, C_DIM, C_OFF = (
    "\033[92m", "\033[91m", "\033[93m", "\033[2m", "\033[0m")
if not sys.stdout.isatty():
    C_OK = C_BAD = C_WARN = C_DIM = C_OFF = ""


# ============================================================ gates

class Gate:
    """A checkable claim about the running system."""
    def __init__(self, label):
        self.label = label
        self.detail = ""
        self.detail_prefix = ""

    def check(self, probe):
        raise NotImplementedError


class TopicRate(Gate):
    """Topic must deliver at least min_hz over a short window."""
    def __init__(self, topic, msg_type, min_hz, window=3.0, validate=None,
                 wait=25.0):
        super().__init__(f"{topic} >= {min_hz} Hz")
        self.topic, self.msg_type = topic, msg_type
        self.min_hz, self.window, self.validate = min_hz, window, validate
        # Wait for the FIRST message before measuring. Fixed settle times are
        # always a guess: odrive.find_any allows 15 s PER device, and one slow
        # ODESC made the drives gate measure a node that was still connecting.
        self.wait = wait

    def check(self, probe):
        waited = probe.wait_for(self.topic, self.msg_type, self.wait)
        if waited is None:
            self.detail = f"no message within {self.wait:.0f}s"
            return False
        msgs = probe.collect(self.topic, self.msg_type, self.window)
        if waited > 1.0:
            self.detail_prefix = f"ready after {waited:.1f}s, "
        else:
            self.detail_prefix = ""
        hz = len(msgs) / self.window
        self.detail = f"{self.detail_prefix}{hz:.1f} Hz, {len(msgs)} msgs"
        if hz < self.min_hz:
            if not msgs:
                self.detail += "  (nothing at all — is the node alive?)"
            return False
        if self.validate:
            ok, why = self.validate(msgs[-1])
            if not ok:
                self.detail += f"  ({why})"
                return False
        return True


class TFExists(Gate):
    def __init__(self, target, source):
        super().__init__(f"TF {target} -> {source}")
        self.target, self.source = target, source

    def check(self, probe):
        t = probe.lookup(self.target, self.source)
        if t is None:
            self.detail = probe.last_tf_error
            return False
        x, y, z = (t.transform.translation.x, t.transform.translation.y,
                   t.transform.translation.z)
        self.detail = f"xyz {x:+.3f} {y:+.3f} {z:+.3f}"
        return True


class TFMoving(Gate):
    """The one a launch file cannot do: transform must CHANGE on its own.

    A frozen-but-present transform passes every other check and yields a
    silent, useless cloud. Samples twice and requires real motion."""
    def __init__(self, target, source, settle=2.5, min_delta=1e-3):
        super().__init__(f"TF {target} -> {source} is MOVING")
        self.target, self.source = target, source
        self.settle, self.min_delta = settle, min_delta

    def check(self, probe):
        a = probe.lookup(self.target, self.source)
        if a is None:
            self.detail = f"no transform: {probe.last_tf_error}"
            return False
        time.sleep(self.settle)
        b = probe.lookup(self.target, self.source)
        if b is None:
            self.detail = "transform vanished mid-check"
            return False
        qa, qb = a.transform.rotation, b.transform.rotation
        drot = max(abs(qa.x - qb.x), abs(qa.y - qb.y),
                   abs(qa.z - qb.z), abs(qa.w - qb.w))
        self.detail = f"rotation delta {drot:.5f} over {self.settle}s"
        if drot < self.min_delta:
            self.detail += "  (FROZEN — stage not turning, or stage TF stale)"
            return False
        return True


class PortOpen(Gate):
    """Something must be LISTENING. rosbridge publishes no topic to check, so
    without this it could die and the run would still report success."""

    def __init__(self, port, host="127.0.0.1", wait=20.0):
        super().__init__(f"tcp/{port} listening")
        self.port, self.host, self.wait = port, host, wait

    def check(self, probe):
        t0 = time.time()
        while time.time() - t0 < self.wait:
            try:
                with socket.create_connection((self.host, self.port), 0.4):
                    self.detail = f"open after {time.time()-t0:.1f}s"
                    return True
            except OSError:
                time.sleep(0.3)
        self.detail = f"nothing listening after {self.wait:.0f}s"
        return False


def rgb_is_640x480(msg):
    ok = (msg.width, msg.height) == (640, 480)
    return ok, (f"{msg.width}x{msg.height} — wrong profile; use "
                f"rgb_camera.color_profile, not .profile" if not ok else "")


def four_wheels(msg):
    n = len(msg.name)
    return n == 4, f"only {n} wheels reporting" if n != 4 else ""


# ============================================================ probe

class Probe(Node):
    """One ROS node the supervisor uses to interrogate everything else.

    THREADING: TransformListener(spin_thread=True) adds THIS node to its own
    SingleThreadedExecutor and spins it in a background thread. So this class
    must never call rclpy.spin_once on itself — two executors on one node drop
    callbacks, which showed up as a static transform that was demonstrably
    being published and yet could not be looked up.

    Everything below therefore sleeps and lets that thread do the work."""

    def __init__(self):
        super().__init__("beetle_up_probe")
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True)
        self.last_tf_error = ""

    def collect(self, topic, msg_type, window):
        got = []
        sub = self.create_subscription(
            msg_type, topic, lambda m: got.append(m), SENSOR_QOS)
        time.sleep(window)              # listener thread delivers
        self.destroy_subscription(sub)
        return got

    def wait_for(self, topic, msg_type, timeout):
        """Seconds until the first message, or None. Lets a gate ride out a
        slow start instead of measuring a node that is not up yet."""
        seen = []
        sub = self.create_subscription(
            msg_type, topic, lambda m: seen.append(m), SENSOR_QOS)
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                if seen:
                    return time.time() - t0
                time.sleep(0.1)
            return None
        finally:
            self.destroy_subscription(sub)

    def lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception as e:
            self.last_tf_error = str(e).split("\n")[0][:110]
            return None


# ============================================================ stages

class Stage:
    def __init__(self, name, cmd=None, cwd=None, gates=(), settle=0.0,
                 note="", optional=False, enabled=True, oneshot=False,
                 match=None):
        self.name, self.cmd, self.cwd = name, cmd, cwd
        self.gates, self.settle, self.note = list(gates), settle, note
        self.optional = optional
        # A `ros2 service call` does its job and exits. Treating that as a
        # crash is wrong — for these, success is returncode 0, not staying up.
        self.oneshot = oneshot
        # pgrep pattern for detecting an instance THIS supervisor did not
        # start. Orphans from a previous session are otherwise invisible —
        # the panel says "stopped" while the node is demonstrably publishing,
        # and starting it again quietly gives you two.
        self.match = match
        # Every stage is BUILT even when not selected, so "I skipped the lidar
        # and changed my mind" is a start command rather than a full restart.
        self.enabled = enabled
        self.proc = None
        self.logfile = None
        self.last_gate = ""

    def external_pids(self):
        """PIDs matching this stage that we do not own."""
        if not self.match:
            return []
        mine = self.proc.pid if self.proc and self.proc.poll() is None else -1
        try:
            out = subprocess.run(["pgrep", "-f", self.match],
                                 capture_output=True, text=True, timeout=4).stdout
        except (OSError, subprocess.SubprocessError):
            return []
        pids = []
        for line in out.split():
            try:
                pid = int(line)
            except ValueError:
                continue
            # pgrep matches its own invocation and our child's whole tree
            if pid in (mine, os.getpid()):
                continue
            try:
                if os.getpgid(pid) == os.getpgid(mine) if mine > 0 else False:
                    continue
            except OSError:
                pass
            pids.append(pid)
        return pids

    def state(self):
        if self.cmd is None:
            return "check"          # a gate-only pseudo-stage
        if self.proc is None:
            return "EXTERNAL" if self.external_pids() else "stopped"
        rc = self.proc.poll()
        if rc is None:
            return "running"
        if self.oneshot:
            return "done" if rc == 0 else "crashed"
        return "exited" if rc == 0 else "crashed"

    def start(self):
        if self.cmd is None:
            return
        LOGS.mkdir(parents=True, exist_ok=True)
        path = LOGS / f"{self.name}.log"
        self.logfile = open(path, "w")
        self.proc = subprocess.Popen(
            self.cmd, cwd=str(self.cwd) if self.cwd else None,
            stdout=self.logfile, stderr=subprocess.STDOUT,
            start_new_session=True)   # own process group, killable as a unit

    def alive(self):
        return self.proc is None or self.proc.poll() is None

    def tail(self, n=18):
        path = LOGS / f"{self.name}.log"
        if not path.exists():
            return "(no log)"
        try:
            lines = path.read_text(errors="replace").splitlines()
        except OSError:
            return "(log unreadable)"
        return "\n".join("    " + ln for ln in lines[-n:]) or "    (log empty)"

    def stop(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except OSError:
                pass
        except OSError:
            pass
        if self.logfile:
            self.logfile.close()


def build_stages(a):
    py = sys.executable or "python3"
    S = []

    # No beacon stage: beetle_agent.py owns discovery and runs from boot,
    # independently of this supervisor. Running beetle_up standalone simply
    # means no beacon, which is fine — you already have a shell open.

    S.append(Stage(
        "camera", match="realsense2_camera",
        cmd=["ros2", "launch", "realsense2_camera", "rs_launch.py",
             "enable_gyro:=true", "enable_accel:=true", "unite_imu_method:=2",
             "enable_depth:=false", "enable_infra:=false",
             "align_depth.enable:=false",
             "rgb_camera.color_profile:=640x480x15"],
        settle=3.0,
        note="RGB + IMU only. Depth + align_depth took load average to 16.4.",
        gates=[TopicRate("/camera/camera/imu", Imu, 100.0),
               TopicRate("/camera/camera/color/camera_info", CameraInfo, 5.0,
                         validate=rgb_is_640x480)]))

    S.append(Stage(
        "drives", match="odesc_tuner_node.py",
        cmd=[py, str(SCRIPTS / "odesc_tuner_node.py"), "--ros-args",
             "-p", f"max_wheel_tps:={a.max_wheel_tps}"],
        cwd=SCRIPTS, settle=2.0,
        note="Must precede odometry. Four ODESC connects take 7-30 s.",
        gates=[TopicRate("/wheel_states", JointState, 10.0,
                         validate=four_wheels, wait=45.0)]))

    S.append(Stage(
        "odom", match="wheel_odom_node.py",
        cmd=[py, str(SCRIPTS / "wheel_odom_node.py"), "--ros-args",
             "-p", f"gyro_scale:={a.gyro_scale}"],
        cwd=SCRIPTS, settle=4.0,
        note="Leave the robot PARKED — gyro bias is learning right now.",
        gates=[TopicRate("/wheel_odom", Odometry, 10.0),
               TFExists("odom", "camera_link")]))

    S.append(Stage(
        "tf_stage_base",
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher",
             "--x", "-0.117", "--y", "0.057", "--z", "0.038",
             "--roll", "0", "--pitch", "0", "--yaw", "0",
             "--frame-id", "camera_link", "--child-frame-id", "stage_base"],
        settle=2.5,
        note="Mounting offset only. Stage ZERO lives in scan_stage_node now.",
        gates=[TFExists("camera_link", "stage_base")]))

    if True:
        S.append(Stage(
            "tf_laser", enabled=not a.drive_only,
            cmd=["ros2", "run", "tf2_ros", "static_transform_publisher",
                 "--x", "0", "--y", "0", "--z", "0.0505",
                 "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
                 "--frame-id", "stage_mount", "--child-frame-id", "laser"],
            settle=2.5,
            note="The 90 degree cant — this is what makes the scan vertical.",
            gates=[]))

        S.append(Stage(
            "lidar", match="rplidar_node", enabled=not a.drive_only,
            cmd=["ros2", "run", "rplidar_ros", "rplidar_node", "--ros-args",
                 "-p", f"serial_port:={a.lidar_port}",
                 "-p", "serial_baudrate:=115200",
                 "-p", "frame_id:=laser"],
            settle=4.0,
            note="/dev/rplidar via udev — ttyUSBn order is not stable.",
            gates=[TopicRate("/scan", LaserScan, 4.0)]))

        S.append(Stage(
            "stage", match="scan_stage_node", enabled=not a.drive_only,
            cmd=["ros2", "run", "scan_stage", "scan_stage_node", "--ros-args",
                 "-p", "lidar_frame:=stage_mount",
                 "-p", f"angle_offset_deg:={a.stage_zero}",
                 "-p", "offset_reset_sec:=5.0"],
            settle=3.0,
            note="offset_reset_sec 5 shrinks the ticks_us wrap damage.",
            gates=[TopicRate("/stage_angle", JointState, 80.0),
                   TFExists("stage_base", "stage_mount")]))

        if True:
            S.append(Stage(
                "spin_up", enabled=a.spin and not a.drive_only, oneshot=True,
                cmd=["ros2", "service", "call", "/stage/spin",
                     "std_srvs/srv/Trigger"],
                settle=3.0, note="Stage spinning up.", gates=[]))

        # THE gate. Everything below camera_link is static or the stage, so a
        # failure here is almost always odom -> camera_link.
        S.append(Stage(
            "FULL CHAIN", enabled=not a.drive_only,
            cmd=None,
            note="odom -> camera_link -> stage_base -> stage_mount -> laser",
            gates=[TFMoving("odom", "laser")] if a.spin
                   else [TFExists("odom", "laser")]))

        if True:
            CLOUDS.mkdir(parents=True, exist_ok=True)
            S.append(Stage(
                "cloud", match="scan_accumulator_node.py", enabled=not (a.no_cloud or a.drive_only),
                cmd=[py, str(SCRIPTS / "scan_accumulator_node.py"), "--ros-args",
                     "-p", "fixed_frame:=odom", "-p", "base_frame:=camera_link",
                     "-p", "mask_frame:=camera_link",
                     "-p", "mask_min_x:=-0.717", "-p", "mask_max_x:=0.133",
                     "-p", "mask_min_y:=-0.243", "-p", "mask_max_y:=0.357",
                     "-p", "mask_min_z:=-0.962", "-p", "mask_max_z:=0.088",
                     "-p", "segments:=8", "-p", "tf_timeout:=0.05",
                     "-p", "voxel_size:=0.05", "-p", "publish_hz:=1.0",
                     "-p", "max_voxels:=200000"],
                cwd=CLOUDS, settle=6.0,
                note="Writes .pcd to the working directory.",
                gates=[]))

        # Slices and thins /cloud_3d for the console. Without it the LIDAR
        # panel's MAP view has nothing to show — the raw cloud is 2.4 MB a
        # message and must never reach a browser.
        if True:
            S.append(Stage(
                "hud_view", match="hud_view_node.py", enabled=not (a.no_cloud or a.drive_only),
                cmd=[py, str(SCRIPTS / "hud_view_node.py"), "--ros-args",
                     "-p", "z_min:=-0.30", "-p", "z_max:=0.60",
                     "-p", "max_points:=4000", "-p", "publish_hz:=1.0"],
                cwd=SCRIPTS, settle=2.0,
                note="z-sliced top-down map for the console LIDAR panel.",
                gates=[], optional=True))

    # BPU detection. Off by default — it is the demo showpiece, not part of
    # driving, and starting it from the NODES panel while the system graph is
    # on screen is a better demonstration than having it up from the start.
    if True:
        S.append(Stage(
            "yolo", match="dnn_node_example", enabled=a.yolo,
            cmd=["ros2", "run", "dnn_node_example", "example", "--ros-args",
                 "-p", "config_file:=config/yolov8workconfig.json",
                 "-p", "feed_type:=1",
                 # shared-memory subscriber fights the RealSense; 0 switches it
                 # to a plain ROS topic subscriber
                 "-p", "is_shared_mem_sub:=0",
                 "-p", "ros_img_topic_name:=/camera/camera/color/image_raw",
                 "-p", "image_width:=640", "-p", "image_height:=480",
                 "-p", "dump_render_img:=0",
                 "-p", "msg_pub_topic_name:=/beetle_detections",
                 "--log-level", "warn"],
            cwd=SCRIPTS, settle=4.0,
            note="YOLOv8n on the BPU. Watch the BPU bar move when it starts.",
            gates=[TopicRate("/beetle_detections", PerceptionTargets, 8.0,
                             wait=30.0)] if PerceptionTargets else [],
            optional=True))

    if True:
        S.append(Stage(
            "rosbridge", match="rosbridge_websocket", enabled=a.console,
            cmd=["ros2", "launch", "rosbridge_server",
                 "rosbridge_websocket_launch.xml",
                 # rosbridge declares these as STRING and parses the
                 # brackets itself. Bare [*] fails yaml.safe_load (* is
                 # an alias marker) and ['*'] yields a STRING_ARRAY the
                 # node rejects. The quotes must survive INTO the value
                 # so yaml resolves it to the plain string [*].
                 "topics_glob:='[*]'", "services_glob:='[*]'",
                 "params_glob:='[*]'"],
            settle=4.0,
            note="Permissive globs — every console knob is a service call.",
            gates=[PortOpen(9090)], optional=True))
        # NOTE: no HTTP server here. beetle_agent.py owns port 8080 and
        # serves the console from boot — starting a second one would just
        # fail to bind, or worse, shadow it.

    if True:
        S.append(Stage(
            "foxglove", match="foxglove_bridge", enabled=a.foxglove,
            cmd=["ros2", "launch", "foxglove_bridge",
                 "foxglove_bridge_launch.xml"],
            settle=3.0, note="Costs ~1 CPU core. Close image panels.",
            gates=[], optional=True))

    return S


# ============================================================ control channel

def control_server(inbox, port=CONTROL_PORT):
    """UDP listener. Requests are QUEUED, never executed here — the main loop
    owns the rclpy context and the child processes, and touching either from
    this thread would race the gate checks."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("127.0.0.1", port))      # localhost only; the agent proxies
    except OSError:
        return
    while True:
        try:
            data, src = sock.recvfrom(4096)
            req = json.loads(data.decode())
        except (OSError, ValueError, UnicodeDecodeError):
            continue
        inbox.put((req, src, sock))


def node_table(stages):
    return [{"name": s.name, "state": s.state(), "enabled": s.enabled,
             "note": s.note, "optional": s.optional,
             "gates": [g.label for g in s.gates],
             "last_gate": s.last_gate,
             "external": s.external_pids(),
             "pid": s.proc.pid if s.proc and s.proc.poll() is None else None}
            for s in stages]


def run_gates(st, probe, verbose=True):
    ok, notes = True, []
    for g in st.gates:
        passed = g.check(probe)
        notes.append(f"{'ok' if passed else 'FAIL'} {g.label} — {g.detail}")
        if verbose:
            mark = f"{C_OK}ok{C_OFF}" if passed else f"{C_BAD}FAIL{C_OFF}"
            print(f"  {mark}  {g.label}  {C_DIM}{g.detail}{C_OFF}")
        ok = ok and passed
    st.last_gate = " | ".join(notes)
    return ok


def handle_control(req, stages, probe):
    """Executed on the main thread. Returns a JSON-able reply."""
    action = req.get("action", "list")
    if action == "list":
        return {"ok": True, "nodes": node_table(stages)}

    name = req.get("name", "")
    st = next((x for x in stages if x.name == name), None)
    if st is None:
        return {"ok": False, "msg": f"no such node: {name}"}
    if st.cmd is None:
        # A gate-only pseudo-stage has nothing to start, but re-checking it on
        # demand is exactly what you want after restarting something upstream.
        if action in ("restart", "start", "check"):
            ok = run_gates(st, probe)
            return {"ok": ok, "msg": st.last_gate or "no gates"}
        return {"ok": False, "msg": "nothing to stop"}

    if action == "stop":
        ext = st.external_pids()
        st.stop()
        st.proc = None
        for pid in ext:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        if ext:
            return {"ok": True,
                    "msg": f"{name} stopped, plus orphan pid(s) "
                           f"{', '.join(map(str, ext))}"}
        return {"ok": True, "msg": f"{name} stopped"}

    if action in ("start", "restart"):
        ext = st.external_pids()
        if ext and action == "start":
            return {"ok": False,
                    "msg": f"{name} is ALREADY RUNNING as pid(s) "
                           f"{', '.join(map(str, ext))} — not started by this "
                           f"supervisor. Stop it first, or you get two."}
        for pid in ext:                      # restart: clear the orphan
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        if ext:
            time.sleep(1.5)
        if st.state() == "running":
            if action == "start":
                return {"ok": False, "msg": f"{name} already running"}
            st.stop()
            st.proc = None
        st.enabled = True
        print(f"\n{C_DIM}control: {action} {name}{C_OFF}")
        st.start()
        time.sleep(max(st.settle, 2.0))
        if not st.alive():
            rc = st.proc.returncode
            if st.oneshot and rc == 0:
                return {"ok": True, "msg": f"{name} completed"}
            return {"ok": False, "msg": f"{name} exited immediately (code {rc})",
                    "tail": st.tail(12)}
        ok = run_gates(st, probe)
        return {"ok": ok, "msg": st.last_gate or f"{name} running",
                "tail": None if ok else st.tail(12)}

    return {"ok": False, "msg": f"unknown action: {action}"}


# ============================================================ driver

def run(a):
    stages = build_stages(a)
    started = []
    rclpy.init()
    probe = Probe()
    failures = []
    degraded = []          # optional stages that failed — NOT a clean run
    inbox = queue.Queue()
    threading.Thread(target=control_server, args=(inbox,), daemon=True).start()

    def shutdown():
        print(f"\n{C_DIM}shutting down…{C_OFF}")
        for s in reversed(started):
            s.stop()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    try:
        width = max(len(s.name) for s in stages)
        selected = [s for s in stages if s.enabled]
        for i, st in enumerate(selected, 1):
            head = f"[{i}/{len(selected)}] {st.name.ljust(width)}"
            print(f"\n{head}  {C_DIM}{st.note}{C_OFF}")
            st.start()
            started.append(st)
            if st.settle:
                time.sleep(st.settle)

            if st.cmd and not st.alive():
                rc = st.proc.returncode
                if st.oneshot and rc == 0:
                    print(f"  {C_OK}ok{C_OFF}  completed  {C_DIM}"
                          f"{st.tail(1).strip()}{C_OFF}")
                else:
                    print(f"  {C_BAD}x process exited immediately "
                          f"(code {rc}){C_OFF}")
                    print(st.tail())
                    if st.optional:
                        degraded.append(st.name)
                    if not resolve(st, a, probe):
                        failures.append(st.name)
                    continue

            ok = True
            for g in st.gates:
                passed = g.check(probe)
                mark = f"{C_OK}ok{C_OFF}" if passed else f"{C_BAD}FAIL{C_OFF}"
                print(f"  {mark}  {g.label}  {C_DIM}{g.detail}{C_OFF}")
                ok = ok and passed
            if not ok:
                print(f"\n  {C_WARN}last lines of {st.name}.log:{C_OFF}")
                print(st.tail())
                if st.optional:
                    degraded.append(st.name)
                if not resolve(st, a, probe):
                    failures.append(st.name)

        banner(a, failures, degraded)
        # Idle loop doubles as the control executor. Requests arrive on a
        # queue from the UDP thread and are executed HERE, on the thread that
        # owns rclpy and the children.
        while True:
            try:
                req, src, sock = inbox.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                reply = handle_control(req, stages, probe)
            except Exception as e:
                reply = {"ok": False, "msg": f"{type(e).__name__}: {e}"}
            try:
                sock.sendto(json.dumps(reply).encode(), src)
            except OSError:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


def resolve(st, a, probe):
    """Gate failed. Retry, skip, or abort."""
    if st.optional:
        print(f"  {C_WARN}optional stage — continuing{C_OFF}")
        return True
    if a.yes:
        print(f"  {C_BAD}aborting (--yes){C_OFF}")
        raise KeyboardInterrupt
    while True:
        try:
            ans = input(f"  {C_WARN}[r]etry  [s]kip  [a]bort > {C_OFF}").strip().lower()
        except EOFError:
            raise KeyboardInterrupt
        if ans.startswith("a"):
            raise KeyboardInterrupt
        if ans.startswith("s"):
            return False
        if ans.startswith("r"):
            st.stop()
            st.start()
            time.sleep(max(st.settle, 3.0))
            ok = all(g.check(probe) for g in st.gates)
            for g in st.gates:
                print(f"    {C_DIM}{g.label}: {g.detail}{C_OFF}")
            if ok:
                print(f"  {C_OK}ok on retry{C_OFF}")
                return True
            print(f"  {C_BAD}still failing{C_OFF}")


def banner(a, failures, degraded=()):
    print("\n" + "=" * 62)
    if failures:
        print(f"{C_WARN}  UP, WITH SKIPPED STAGES: {', '.join(failures)}{C_OFF}")
    elif degraded:
        # "all gates passed" while rosbridge lay dead was actively misleading.
        print(f"{C_WARN}  UP, BUT DEGRADED: {', '.join(degraded)} failed{C_OFF}")
        if "rosbridge" in degraded:
            print(f"{C_WARN}  -> the console will load but never connect{C_OFF}")
    else:
        print(f"{C_OK}  BEETLE UP — all gates passed{C_OFF}")
    print(f"""
  Deck:     python3 deck_teleop.py --host <X5_IP>
  Console:  http://<X5_IP>:8080/beetle_console.html
  Logs:     tail -f {LOGS}/<name>.log

  Ctrl-C here stops everything.""")
    print("=" * 62)


def main():
    p = argparse.ArgumentParser(description="Gated bringup for Beetle.")
    p.add_argument("--drive-only", action="store_true",
                   help="camera + drives + odometry, no lidar/stage/cloud")
    p.add_argument("--no-cloud", action="store_true",
                   help="lidar and stage, but no accumulator")
    p.add_argument("--spin", action="store_true",
                   help="start the stage turning; also upgrades the full-chain "
                        "gate from EXISTS to MOVING")
    p.add_argument("--console", action="store_true", help="rosbridge + HTTP server")
    p.add_argument("--foxglove", action="store_true")
    p.add_argument("--yolo", action="store_true",
                   help="BPU detection at startup; otherwise start it from "
                        "the console's NODES panel")
    p.add_argument("--gyro-scale", default="1.0")
    p.add_argument("--stage-zero", default="0.0")
    p.add_argument("--max-wheel-tps", default="0.6")
    p.add_argument("--lidar-port", default="/dev/rplidar")
    p.add_argument("--console-dir", default="/root/beetle_hud")
    p.add_argument("--no-beacon", action="store_true",
                   help="skip the discovery beacon")
    p.add_argument("--yes", action="store_true",
                   help="non-interactive: abort on any gate failure")
    run(p.parse_args())


if __name__ == "__main__":
    main()
