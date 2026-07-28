#!/usr/bin/env python3
"""
odesc_teleop_node.py — runs on the X5.

/cmd_vel (+ /estop) -> four ODESC V4.2 over USB, skid-steer mix,
closed-loop VELOCITY control only (never torque — that's the mode that
produced the runaway, and it stays off until the hardware deadman
exists).

This is the minimal cmd_mux from the design doc: one node owns the
path to the motors, every velocity source funnels through /cmd_vel,
and the soft e-stop rides here.

Safety layers, in order:
  1. Deck deadman (upstream) — release = zeros on /cmd_vel
  2. This node's command watchdog — no /cmd_vel for cmd_timeout -> zeros
  3. /estop topic (std_msgs/Bool) — True latches: zeros + drop to IDLE.
     Publish False to clear, node re-enters closed loop.
  4. ODrive firmware watchdog — enabled at runtime, fed every tick; if
     this node or the USB link dies, the axis faults to idle on its own.
  5. Wheel-speed clamp (max_wheel_tps, default 2.0 t/s, under vel_limit=5)
  6. Hardware e-stop — yours, outside software, always.

Wheel assignment: the doc's open item. Fill in the four serials below
per position (decimal, straight from odesc_quick_reference.md board
inventory), label the boards physically while you're at it.

First run is WHEELS-UP ON THE CART: check each wheel's direction sign,
then put it on the floor.

Run:
  source /opt/tros/humble/setup.bash
  pip install odrive==0.5.4   # if not already the env you use
  python3 odesc_teleop_node.py

  # calibration runs only — adds cumulative wheel position (rad) to
  # /wheel_states for distance measurement by differencing:
  python3 odesc_teleop_node.py --ros-args -p publish_position:=true
"""

import json
import threading
import time

import odrive  # 0.5.4 — matches firmware 0.5.6 per the quick reference
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

TWO_PI = 6.283185307179586

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# position -> (serial decimal string, direction sign)
# AUDITED 2026-07-22 (creep test, wheels-up): these are Isaac's verified
# physical positions. Signs verified at motor level: +vel = forward on
# left wheels, backward on right (mirrored mounts), so right side = -1.
# node IDs for reference: LF=3, LR=1, RF=0, RR=2.
# 2026-07-28: right_rear board REPLACED — a USB cable worked loose,
# wrapped a motor and killed the original (serial 58803294777653).
# New board 63063866028595 / hex 395B33553233, set to node_id 2.
# Direction sign is unchanged: it describes the mirrored mount, not
# the board. Still verify it wheels-up before the floor.
WHEELS = {
    "left_front":  ("62041683145013", +1),
    "left_rear":   ("58807586992437", +1),
    "right_front": ("62041683472693", -1),
    "right_rear":  ("63063866028595", -1),
}


class OdescTeleop(Node):
    def __init__(self):
        super().__init__("odesc_teleop")
        self.declare_parameter("wheel_radius", 0.0765)   # measured, see below
        self.declare_parameter("track_width", 0.375)      # m, measured
        self.declare_parameter("max_wheel_tps", 2.0)     # turns/s clamp
        self.declare_parameter("cmd_timeout", 0.3)       # s
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("use_fw_watchdog", True)
        self.declare_parameter("fw_watchdog_timeout", 1.0)
        # Partial drivetrain = veering robot, not a degraded robot. Refuse
        # to run unless explicitly overridden for bench work.
        self.declare_parameter("allow_partial", False)
        # CALIBRATION SLOT — off by default. Adds pos_estimate to
        # /wheel_states so a run's wheel rotation can be measured by
        # DIFFERENCING position instead of integrating 20 Hz velocity
        # samples through accel/decel ramps. Costs 4 more fibre reads per
        # tick (8 -> 12), so it stays off until calibration asks for it.
        self.declare_parameter("publish_position", False)

        self.r = self.get_parameter("wheel_radius").value
        self.track = self.get_parameter("track_width").value
        self.max_tps = self.get_parameter("max_wheel_tps").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.use_wd = self.get_parameter("use_fw_watchdog").value
        self.wd_timeout = self.get_parameter("fw_watchdog_timeout").value

        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)          # vx, wz
        self.last_cmd_time = None
        self.estopped = False

        self.devs = {}                  # position -> (odrive obj, sign)
        self.connect_all()
        if not self.enable_all():
            if self.get_parameter("allow_partial").value:
                self.get_logger().warn(
                    "PARTIAL DRIVETRAIN — driving anyway (allow_partial). "
                    "Bench use only; do not floor-drive like this.")
            else:
                self.get_logger().fatal(
                    "Not all wheels reached closed loop — refusing to "
                    "drive a partial drivetrain (it veers, it doesn't "
                    "degrade). Fix the fault or set allow_partial:=true "
                    "for bench work.")
                self.all_idle()
                raise SystemExit(1)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)

        # Wheel telemetry: same numbers CAN would carry (Encoder Estimates),
        # polled over the USB/fibre link we already hold. Published, NOT
        # fused into odometry — skid-steer rotation is scrub-corrupted, so
        # this is for eyes/HUD/bags (cmd-vs-actual, fault spotting).
        self.joint_pub = self.create_publisher(JointState, "/wheel_states", 10)
        self.status_pub = self.create_publisher(String, "/wheel_status", 10)
        self.targets = {p: 0.0 for p in WHEELS}    # last commanded turns/s
        self.actuals = {p: 0.0 for p in WHEELS}    # last read turns/s
        # Cumulative turns since the DRIVER booted, sign-corrected to robot
        # convention. The absolute value is arbitrary — only differences
        # across a run are meaningful, which is all calibration needs.
        self.pub_pos = self.get_parameter("publish_position").value
        self.positions = {p: 0.0 for p in WHEELS}
        self.telemetry = {p: {} for p in WHEELS}   # slow-rate extras
        self.prev_fault = {p: False for p in WHEELS}
        # Watchdog feeding lives on this 5 Hz timer, not the control tick:
        # 0.2 s period vs the 1.0 s timeout is ample, and it keeps the
        # 20 Hz tick at 8 fibre calls instead of 12.
        self.status_timer = self.create_timer(0.2, self.status_tick)

        rate = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("Driving on /cmd_vel. /estop Bool true=stop.")

    # ---- ODESC handling ----------------------------------------------------

    def connect_all(self):
        for pos, (serial_dec, sign) in WHEELS.items():
            hexser = format(int(serial_dec), "X")
            self.get_logger().info(f"Connecting {pos} (serial {hexser})...")
            d = odrive.find_any(serial_number=hexser, timeout=15)
            # "Not a genuine ODrive!" warning on connect is expected/harmless
            d.clear_errors()
            self.devs[pos] = (d, sign)
            self.get_logger().info(
                f"  {pos}: vbus={d.vbus_voltage:.1f}V "
                f"state={d.axis0.current_state}")

    def enable_all(self):
        # Enter closed loop FIRST, arm the watchdog LAST. The firmware
        # checks the watchdog every control cycle (~125 us) while fibre
        # writes land ~1 ms apart — and enabling does NOT reset the
        # internal timer. Enable-before-feed therefore trips a never-fed
        # timer in the enable->feed gap (0x800 on all four boards,
        # 2026-07-22 bringup). Arm only once the axis is up and the 5 Hz
        # feeder is about to run.
        for pos, (d, _) in self.devs.items():
            d.axis0.config.enable_watchdog = False
            d.clear_errors()
            d.axis0.controller.input_vel = 0.0
            d.axis0.requested_state = AXIS_STATE_CLOSED_LOOP
        time.sleep(0.3)
        all_ok = True
        for pos, (d, _) in self.devs.items():
            if d.axis0.current_state != AXIS_STATE_CLOSED_LOOP:
                all_ok = False
                self.get_logger().error(
                    f"{pos} failed to enter closed loop "
                    f"(state={d.axis0.current_state}, "
                    f"axis_err={hex(d.axis0.error)}, "
                    f"motor_err={hex(d.axis0.motor.error)}, "
                    f"enc_err={hex(d.axis0.encoder.error)})")
        if self.use_wd and all_ok:
            for pos, (d, _) in self.devs.items():
                d.axis0.config.watchdog_timeout = self.wd_timeout
                d.axis0.watchdog_feed()
                d.axis0.config.enable_watchdog = True
                d.axis0.watchdog_feed()
            self.get_logger().info(
                f"Firmware watchdogs armed ({self.wd_timeout}s)")
        return all_ok

    def all_idle(self):
        for _, (d, _) in self.devs.items():
            try:
                d.axis0.controller.input_vel = 0.0
                d.axis0.requested_state = AXIS_STATE_IDLE
                if self.use_wd:
                    d.axis0.config.enable_watchdog = False
                    d.clear_errors()  # a watchdog trip latches; clean exit
            except Exception as e:
                self.get_logger().error(f"idle failed: {e}")

    # ---- ROS callbacks -----------------------------------------------------

    def on_cmd(self, msg: Twist):
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)
            self.last_cmd_time = time.monotonic()

    def on_estop(self, msg: Bool):
        if msg.data and not self.estopped:
            self.estopped = True
            self.get_logger().warn("SOFT E-STOP — motors to IDLE")
            self.all_idle()
        elif not msg.data and self.estopped:
            self.estopped = False
            self.get_logger().warn("E-stop cleared — re-enabling")
            for _, (d, _) in self.devs.items():
                d.clear_errors()
            self.enable_all()

    # ---- control loop ------------------------------------------------------

    def tick(self):
        if self.estopped:
            return

        with self.lock:
            vx, wz = self.cmd
            stale = (self.last_cmd_time is None or
                     time.monotonic() - self.last_cmd_time > self.cmd_timeout)
        if stale:
            vx, wz = 0.0, 0.0

        circ = 2 * 3.141592653589793 * self.r
        left_tps = (vx - wz * self.track / 2.0) / circ
        right_tps = (vx + wz * self.track / 2.0) / circ
        # Clamp PROPORTIONALLY, never per-side. Clamping each side against
        # the limit independently destroys the differential: at full throttle
        # both sides saturate to the SAME value and the robot cannot steer at
        # all, while turning on the spot still works because forward is zero.
        # Scaling both by one factor caps the fastest wheel and keeps the
        # left/right ratio — so the commanded turn survives.
        peak = max(abs(left_tps), abs(right_tps))
        if peak > self.max_tps and peak > 0.0:
            k = self.max_tps / peak
            left_tps *= k
            right_tps *= k

        targets = {"left_front": left_tps, "left_rear": left_tps,
                   "right_front": right_tps, "right_rear": right_tps}

        for pos, (d, sign) in self.devs.items():
            try:
                d.axis0.controller.input_vel = sign * targets[pos]
                self.targets[pos] = targets[pos]
                # read back actual wheel velocity (turns/s, sign-corrected
                # to robot convention so cmd and actual compare directly)
                self.actuals[pos] = sign * d.axis0.encoder.vel_estimate
                if self.pub_pos:
                    self.positions[pos] = sign * d.axis0.encoder.pos_estimate
            except Exception as e:
                self.get_logger().error(f"{pos} write failed: {e}")

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"wheel_{p}" for p in self.devs]
        js.velocity = [self.actuals[p] * TWO_PI for p in self.devs]  # rad/s
        if self.pub_pos:
            js.position = [self.positions[p] * TWO_PI for p in self.devs]  # rad
        self.joint_pub.publish(js)

    def status_tick(self):
        """5 Hz: watchdog feed + slow telemetry (state, current, faults)."""
        report = {}
        for pos, (d, sign) in self.devs.items():
            entry = {"cmd": round(self.targets[pos], 3),
                     "vel": round(self.actuals[pos], 3)}
            try:
                if self.use_wd and not self.estopped:
                    d.axis0.watchdog_feed()
                state = d.axis0.current_state
                entry["state"] = state
                entry["iq"] = round(
                    d.axis0.motor.current_control.Iq_measured, 2)
                # a wheel that should be driving but isn't = fault
                healthy = (state == AXIS_STATE_CLOSED_LOOP)
                fault = not healthy and not self.estopped
                entry["fault"] = not healthy
                if not healthy:
                    entry["err"] = hex(d.axis0.error)
                if fault:
                    if not self.prev_fault[pos]:
                        self.get_logger().error(
                            f"{pos} dropped out mid-run: state={state} "
                            f"axis_err={entry['err']} — LATCHING STOP "
                            f"(clear: /estop true then false)")
                        # a 3-wheel skid steer veers; stop everything
                        self.estopped = True
                        self.all_idle()
                    self.prev_fault[pos] = True
                elif healthy and self.prev_fault[pos]:
                    self.get_logger().info(f"{pos} recovered")
                    self.prev_fault[pos] = False
            except Exception as e:
                entry["fault"] = True
                entry["err"] = f"usb: {e}"
            report[pos] = entry
        msg = String()
        msg.data = json.dumps(report)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = OdescTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down — motors to IDLE")
        node.all_idle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()