#!/usr/bin/env python3
"""
odesc_tuner_node.py — X5-side companion to deck_tuner.py.

Same job as odesc_teleop_node.py (UDP -> skid-steer mix -> four ODESCs
in velocity mode, with the same safety layers), plus:

  * accepts live parameter writes from the Deck and applies them to all
    four boards
  * streams per-wheel telemetry back (cmd, actual, Iq, vbus, state,
    faults) so the Deck can render a tuning console
  * handles save / revert / clear / reboot commands

Deliberately a SEPARATE file from odesc_teleop_node.py: that one is the
known-good driving node, and this one is a bench tool that reboots
drivers and writes flash. Keep the boring one boring.

Run (X5):
    source /opt/tros/humble/setup.bash
    python3 odesc_tuner_node.py

Publishes /wheel_states, /wheel_status and subscribes /cmd_vel, /estop
exactly like the teleop node, so the HUD and Foxglove still work.
"""

import json
import socket
import threading
import time

import odrive
import odrive.enums as odenums
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
TWO_PI = 6.283185307179586

# AUDITED 2026-07-22 — see odesc_teleop_node.py
WHEELS = {
    "left_front":  ("62041683145013", +1),
    "left_rear":   ("58807586992437", +1),
    "right_front": ("62041683472693", -1),
    "right_rear":  ("63063866028595", -1),
}

# param key -> attribute path relative to the odrive device object
PARAM_PATHS = {
    "vel_gain":                "axis0.controller.config.vel_gain",
    "vel_integrator_gain":     "axis0.controller.config.vel_integrator_gain",
    "vel_limit":               "axis0.controller.config.vel_limit",
    "current_lim":             "axis0.motor.config.current_lim",
    "torque_constant":         "axis0.motor.config.torque_constant",
    "dc_max_positive_current": "config.dc_max_positive_current",
    "dc_max_negative_current": "config.dc_max_negative_current",
}


def get_path(obj, path):
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


def set_path(obj, path, value):
    parts = path.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


# Error registers are BIT FIELDS — 0x340 is three bits set at once, not one
# code. Names come from the installed odrive package, so they stay correct
# across firmware/clone variations instead of drifting from a hardcoded list.
ERROR_REGISTERS = [
    ("AXIS",       "axis0.error",            "AXIS_ERROR_"),
    ("MOTOR",      "axis0.motor.error",      "MOTOR_ERROR_"),
    ("ENCODER",    "axis0.encoder.error",    "ENCODER_ERROR_"),
    ("CONTROLLER", "axis0.controller.error", "CONTROLLER_ERROR_"),
]


def decode_bits(value, prefix):
    """Return the set bit names for one error register."""
    if not value:
        return []
    names = []
    for k, v in odenums.__dict__.items():
        if (k.startswith(prefix) and isinstance(v, int)
                and v and (value & v) == v):
            names.append(k[len(prefix):])
    if not names:
        names.append(f"UNKNOWN({hex(value)})")
    return names


def decode_device_errors(d):
    """All four registers -> ('0x340', ['AXIS: MOTOR_FAILED', ...])."""
    labels, axis_hex = [], "0x0"
    for reg_name, path, prefix in ERROR_REGISTERS:
        try:
            value = int(get_path(d, path))
        except Exception:
            continue
        if reg_name == "AXIS":
            axis_hex = hex(value)
        for bit in decode_bits(value, prefix):
            labels.append(f"{reg_name}: {bit}")
    return axis_hex, labels


class OdescTuner(Node):
    def __init__(self):
        super().__init__("odesc_tuner")
        self.declare_parameter("wheel_radius", 0.0765)
        self.declare_parameter("track_width", 0.375)
        self.declare_parameter("cmd_timeout", 0.3)
        # Wheel-speed clamp — safety layer 5, matching odesc_teleop_node.
        # This was MISSING here: the parameter was never declared and the
        # clamp never applied, so `-p max_wheel_tps:=0.6` was silently
        # ignored (ROS 2 accepts undeclared params on the command line
        # without error) and the drives ran at whatever the Deck asked for.
        self.declare_parameter("max_wheel_tps", 2.0)     # turns/s clamp
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("deck_port", 5006)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("allow_partial", True)  # bench tool

        self.r = self.get_parameter("wheel_radius").value
        self.track = self.get_parameter("track_width").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.max_tps = self.get_parameter("max_wheel_tps").value
        self.deck_port = self.get_parameter("deck_port").value

        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        # Read from the Deck packet and published, so the console can show
        # whether the stick is actually being held. Previously consumed and
        # thrown away, which made "why won't it drive" hard to answer.
        self.deadman = False
        self.estopped = False
        self.busy = ""
        self.logbuf = []
        self.deck_addr = None
        self.targets = {p: 0.0 for p in WHEELS}
        self.actuals = {p: 0.0 for p in WHEELS}
        self.telemetry = {p: {} for p in WHEELS}
        self.applied = {}
        self.saved = {}
        self.disagree = []
        self.save_serial = 0     # increments on each completed save

        self.devs = {}
        self.connect_all()
        self.enable_all()
        self.read_params(initial=True)

        port = self.get_parameter("udp_port").value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.joint_pub = self.create_publisher(JointState, "/wheel_states", 10)
        self.status_pub = self.create_publisher(String, "/wheel_status", 10)
        # Same action as the UDP "clear" command, reachable over rosbridge
        # so the console can reset drives without the UDP channel.
        self.create_service(Trigger, "~/clear_errors", self.on_clear_errors_srv)
        self.create_service(Trigger, "~/reboot_drives", self.on_reboot_drives_srv)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)

        rate = self.get_parameter("rate_hz").value
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(0.1, self.telemetry_tick)   # 10 Hz to the Deck
        self.log("tuner ready")

    # ---- logging ----------------------------------------------------------

    def log(self, msg):
        self.get_logger().info(msg)
        self.logbuf.append(msg)
        self.logbuf = self.logbuf[-6:]

    # ---- ODESC ------------------------------------------------------------

    def connect_all(self):
        for pos, (serial_dec, sign) in WHEELS.items():
            hexser = format(int(serial_dec), "X")
            self.get_logger().info(f"Connecting {pos} ({hexser})...")
            d = odrive.find_any(serial_number=hexser, timeout=15)
            d.clear_errors()
            self.devs[pos] = (d, sign)

    def enable_all(self):
        ok = True
        for pos, (d, _) in self.devs.items():
            try:
                d.axis0.config.enable_watchdog = False
                d.clear_errors()
                d.axis0.controller.input_vel = 0.0
                d.axis0.requested_state = AXIS_STATE_CLOSED_LOOP
            except Exception as e:
                self.log(f"{pos} enable failed: {e}")
                ok = False
        time.sleep(0.3)
        for pos, (d, _) in self.devs.items():
            try:
                if d.axis0.current_state != AXIS_STATE_CLOSED_LOOP:
                    self.log(f"{pos} not in closed loop "
                             f"(err {hex(d.axis0.error)})")
                    ok = False
            except Exception:
                ok = False
        return ok

    def read_params(self, initial=False):
        """Read every param from all boards; flag disagreement."""
        applied, disagree = {}, []
        for key, path in PARAM_PATHS.items():
            vals = []
            for pos, (d, _) in self.devs.items():
                try:
                    vals.append(float(get_path(d, path)))
                except Exception:
                    pass
            if not vals:
                continue
            applied[key] = vals[0]
            if max(vals) - min(vals) > 1e-6:
                disagree.append(key)
        self.applied = applied
        self.disagree = disagree
        if initial:
            self.saved = dict(applied)   # flash == live at startup

    def apply_params(self, params):
        for key, value in params.items():
            path = PARAM_PATHS.get(key)
            if not path:
                continue
            for pos, (d, _) in self.devs.items():
                try:
                    set_path(d, path, float(value))
                except Exception as e:
                    self.log(f"{key} rejected on {pos}: {e}")
        self.read_params()

    # ---- long-running commands (threaded, non-blocking) -------------------

    def on_reboot_drives_srv(self, request, response):
        """Power-cycles the ODrive firmware and reconnects. The tier above
        clearing errors: use it when a drive is wedged rather than merely
        faulted. Does nothing for a drive that has fallen off USB entirely —
        that needs the agent's USB reset, because this node cannot answer a
        service call while it is blocked inside find_any()."""
        if self.busy:
            response.success = False
            response.message = f"busy: {self.busy}"
            return response
        self.run_command("reboot")
        response.success = True
        response.message = "rebooting drivers, ~5 s"
        return response

    def on_clear_errors_srv(self, request, response):
        """ROS service mirror of the UDP "clear" command, so the rosbridge
        console can reset the drives without the UDP channel."""
        if self.busy:
            response.success = False
            response.message = f"busy: {self.busy}"
            return response
        self.run_command("clear")
        response.success = True
        response.message = "clearing errors and re-entering closed loop"
        return response

    def run_command(self, cmd):
        if self.busy:
            return
        threading.Thread(target=self._command_worker, args=(cmd,),
                         daemon=True).start()

    def _command_worker(self, cmd):
        try:
            if cmd == "clear":
                self.busy = "clearing errors..."
                for pos, (d, _) in self.devs.items():
                    d.axis0.controller.input_vel = 0.0
                    d.clear_errors()
                self.enable_all()
                self.log("errors cleared, closed loop re-entered")

            elif cmd == "save":
                self.busy = "saving (idle -> flash -> reboot)..."
                # Snapshot what we're about to commit. Re-reading after the
                # save races the boards' reboot, so trust the values we
                # verified were applied rather than whatever a fresh read
                # catches mid-reconnect.
                self.read_params()
                committing = dict(self.applied)
                for pos, (d, _) in self.devs.items():
                    d.axis0.controller.input_vel = 0.0
                    d.axis0.requested_state = AXIS_STATE_IDLE
                time.sleep(0.3)
                for i, pos in enumerate(list(self.devs)):
                    d, sign = self.devs[pos]
                    self.busy = f"saving {pos} ({i+1}/4)..."
                    try:
                        d.save_configuration()
                    except Exception:
                        pass          # save reboots the board mid-request
                    time.sleep(3.0)
                self.busy = "reconnecting after save..."
                self.reconnect_all()
                self.read_params()
                # flash now holds what we committed; verify against the
                # re-read and warn on any value that didn't stick
                self.saved = dict(committing)
                for k, v in committing.items():
                    live = self.applied.get(k)
                    if live is not None and abs(live - v) > 1e-6:
                        self.log(f"WARNING {k} reverted: {v:g} -> {live:g}")
                        self.saved[k] = live
                self.save_serial += 1
                self.log("saved to flash on all four boards")

            elif cmd == "reboot":
                self.busy = "rebooting drivers..."
                for pos, (d, _) in self.devs.items():
                    try:
                        d.reboot()
                    except Exception:
                        pass
                time.sleep(3.0)
                self.reconnect_all()
                self.read_params()
                self.log("drivers rebooted and reconnected")
        except Exception as e:
            self.log(f"command {cmd} failed: {e}")
        finally:
            self.busy = ""

    def reconnect_all(self):
        self.devs = {}
        self.connect_all()
        self.enable_all()

    # ---- ROS callbacks ----------------------------------------------------

    def on_cmd(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)
            self.last_cmd_time = time.monotonic()

    def on_estop(self, msg):
        if msg.data and not self.estopped:
            self.estopped = True
            self.idle_all()
            self.log("SOFT E-STOP")
        elif not msg.data and self.estopped:
            self.estopped = False
            self.enable_all()
            self.log("e-stop cleared")

    def idle_all(self):
        for pos, (d, _) in self.devs.items():
            try:
                d.axis0.controller.input_vel = 0.0
                d.axis0.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass

    # ---- UDP --------------------------------------------------------------

    def poll_udp(self):
        newest, addr = None, None
        while True:
            try:
                data, a = self.sock.recvfrom(4096)
                newest, addr = data, a
            except BlockingIOError:
                break
        if newest is None:
            return
        self.deck_addr = (addr[0], self.deck_port)
        try:
            d = json.loads(newest.decode())
        except ValueError:
            return
        with self.lock:
            self.deadman = bool(d.get("deadman"))
            if self.deadman:
                self.cmd = (float(d.get("vx", 0.0)), float(d.get("wz", 0.0)))
            else:
                self.cmd = (0.0, 0.0)
            self.last_cmd_time = time.monotonic()
        if d.get("params") and not self.busy:
            self.apply_params(d["params"])
        if d.get("command"):
            self.run_command(d["command"])

    def telemetry_tick(self):
        if not self.deck_addr:
            return
        payload = {
            "wheels": self.telemetry,
            "applied": self.applied,
            "saved": self.saved,
            "disagree": self.disagree,
            "busy": self.busy,
            "save_serial": self.save_serial,
            "log": self.logbuf[-3:],
        }
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.deck_addr)
        except OSError:
            pass

    # ---- control loop -----------------------------------------------------

    def tick(self):
        self.poll_udp()
        if self.busy or self.estopped:
            return

        with self.lock:
            vx, wz = self.cmd
            stale = (self.last_cmd_time is None or
                     time.monotonic() - self.last_cmd_time > self.cmd_timeout)
        if stale:
            vx, wz = 0.0, 0.0

        circ = TWO_PI * self.r
        left = (vx - wz * self.track / 2.0) / circ
        right = (vx + wz * self.track / 2.0) / circ
        # Live: re-read so the console can change speed without a restart.
        mt = float(self.get_parameter("max_wheel_tps").value)
        # Clamp PROPORTIONALLY, never per-side. Clamping each side against
        # the limit independently destroys the differential: at full throttle
        # both sides saturate to the SAME value and the robot cannot steer at
        # all, while turning on the spot still works because forward is zero.
        # Scaling both by one factor caps the fastest wheel and keeps the
        # left/right ratio — so the commanded turn survives.
        peak = max(abs(left), abs(right))
        if peak > mt and peak > 0.0:
            k = mt / peak
            left *= k
            right *= k
        targets = {"left_front": left, "left_rear": left,
                   "right_front": right, "right_rear": right}

        report = {}
        for pos, (d, sign) in self.devs.items():
            entry = {}
            try:
                d.axis0.controller.input_vel = sign * targets[pos]
                self.targets[pos] = targets[pos]
                self.actuals[pos] = sign * d.axis0.encoder.vel_estimate
                state = d.axis0.current_state
                entry = {
                    "cmd": round(targets[pos], 3),
                    "vel": round(self.actuals[pos], 3),
                    "iq": round(d.axis0.motor.current_control.Iq_measured, 2),
                    "vbus": round(d.vbus_voltage, 1),
                    "state": state,
                    "fault": state != AXIS_STATE_CLOSED_LOOP,
                }
                if entry["fault"]:
                    err_hex, err_names = decode_device_errors(d)
                    entry["err"] = err_hex
                    entry["err_names"] = err_names
            except Exception as e:
                entry = {"fault": True, "err": f"usb:{e}"}
            report[pos] = entry
        self.telemetry = report

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"wheel_{p}" for p in self.devs]
        js.velocity = [self.actuals[p] * TWO_PI for p in self.devs]
        self.joint_pub.publish(js)

        msg = String()
        # Deck link state, alongside the per-wheel entries. Consumers index
        # by wheel name, so an extra key is inert to them.
        with self.lock:
            last = self.last_cmd_time
            dm = self.deadman
        age = None if last is None else round(time.monotonic() - last, 2)
        report["link"] = {
            "up": age is not None and age <= self.cmd_timeout,
            "deadman": bool(dm),
            "age": age,
        }
        msg.data = json.dumps(report)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = OdescTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("shutting down — motors to IDLE")
        node.idle_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
