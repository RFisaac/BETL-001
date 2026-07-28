#!/usr/bin/env python3
"""
drive_log.py — log /wheel_states and /wheel_odom to ONE time-aligned CSV.

Both topics come off the same 20 Hz chain, so one row per /wheel_states
message with the most recent odometry attached is aligned to within a
single tick.

THE QUESTION THIS ANSWERS
  Translation isn't accumulating. Two candidates:
    (a) the wheels report motion, and wheel_odom fails to integrate it
    (b) the wheels don't report sustained motion in the first place
  This recomputes forward velocity from the raw wheel velocities using
  the SAME deadband-and-average logic wheel_odom uses, integrates it
  independently, and puts that next to what wheel_odom actually
  produced. If dist_wheels climbs while odom_path doesn't, it's (a).
  If neither climbs while you're driving, it's (b).

Run (X5), from wherever you want the CSV:
    python3 /userdata/drive_log.py

    # match a deadband you've changed live:
    python3 /userdata/drive_log.py --ros-args -p velocity_deadband:=0.8

Prints a one-line summary at 1 Hz so there IS something to watch, while
writing the full 20 Hz to file. Ctrl-C prints totals.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

COLS = ["t", "lf", "lr", "rf", "rr",
        "vl", "vr", "v_wheels", "dist_wheels", "parked",
        "odom_x", "odom_y", "odom_yaw_deg", "odom_v", "odom_w",
        "odom_path", "odom_disp"]


class DriveLog(Node):
    def __init__(self):
        super().__init__("drive_log")

        self.declare_parameter("wheel_states_topic", "/wheel_states")
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("out", "")            # blank -> timestamped
        # Keep these matched to wheel_odom_node or the comparison is void.
        self.declare_parameter("wheel_radius", 0.0765)
        self.declare_parameter("velocity_deadband", 0.35)
        self.declare_parameter("print_hz", 1.0)

        gp = lambda n: self.get_parameter(n).value
        self.r = float(gp("wheel_radius"))
        self.deadband = float(gp("velocity_deadband"))

        path = gp("out") or f"drive_log_{int(time.time())}.csv"
        self.path = path
        self.f = open(path, "w")
        self.f.write(",".join(COLS) + "\n")

        self.odom = None            # (x, y, yaw, v, w)
        self.last_t = None
        self.dist_wheels = 0.0      # independent integral of |v_wheels|
        self.odom_path = 0.0        # arc length walked by the odom pose
        self.last_xy = None
        # wheel_odom has usually been running a while and its pose is
        # already far from origin. Anchor to where WE started so the
        # displacement column means "moved during this log".
        self.start_xy = None
        self.n = 0
        self.t0 = None

        self.create_subscription(
            JointState, gp("wheel_states_topic"), self.on_wheels, 20)
        self.create_subscription(
            Odometry, gp("odom_topic"), self.on_odom, 20)
        self.create_timer(1.0 / max(0.1, float(gp("print_hz"))), self.tick)

        self.get_logger().info(
            f"logging -> {path}  (r={self.r}, deadband={self.deadband})")

    # ---------- inputs ----------

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_xy is None:
            self.start_xy = (x, y)
        if self.last_xy is not None:
            self.odom_path += math.hypot(x - self.last_xy[0],
                                         y - self.last_xy[1])
        self.last_xy = (x, y)
        self.odom = (x, y, yaw,
                     msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_wheels(self, msg):
        if not msg.name or not msg.velocity:
            return

        # Same reduction wheel_odom does, so the two are comparable.
        raw = {}
        left, right = [], []
        for name, vel in zip(msg.name, msg.velocity):
            raw[name] = vel
            v = 0.0 if abs(vel) < self.deadband else vel
            if "left" in name:
                left.append(v)
            elif "right" in name:
                right.append(v)
        if not left or not right:
            return

        vl = (sum(left) / len(left)) * self.r
        vr = (sum(right) / len(right)) * self.r
        parked = (abs(vl) < 1e-6 and abs(vr) < 1e-6)
        v = 0.5 * (vl + vr)

        t = Time.from_msg(msg.header.stamp)
        t_s = t.nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t_s
        if self.last_t is not None:
            dt = (t - self.last_t).nanoseconds * 1e-9
            if 0.0 < dt <= 1.0:
                self.dist_wheels += abs(v) * dt
        self.last_t = t

        o = self.odom or (0.0, 0.0, 0.0, 0.0, 0.0)
        sx, sy = self.start_xy or (o[0], o[1])
        disp = math.hypot(o[0] - sx, o[1] - sy)

        row = [f"{t_s - self.t0:.4f}",
               f"{raw.get('wheel_left_front', 0.0):.4f}",
               f"{raw.get('wheel_left_rear', 0.0):.4f}",
               f"{raw.get('wheel_right_front', 0.0):.4f}",
               f"{raw.get('wheel_right_rear', 0.0):.4f}",
               f"{vl:.4f}", f"{vr:.4f}", f"{v:.4f}",
               f"{self.dist_wheels:.4f}", "1" if parked else "0",
               f"{o[0]:.4f}", f"{o[1]:.4f}", f"{math.degrees(o[2]):.2f}",
               f"{o[3]:.4f}", f"{o[4]:.4f}",
               f"{self.odom_path:.4f}", f"{disp:.4f}"]
        self.f.write(",".join(row) + "\n")
        self.n += 1
        if self.n % 40 == 0:
            self.f.flush()

    # ---------- output ----------

    def tick(self):
        if self.odom is None:
            self.get_logger().info("waiting for /wheel_odom ...")
            return
        x, y, yaw, v, w = self.odom
        sx, sy = self.start_xy or (x, y)
        self.get_logger().info(
            f"n={self.n} wheels_dist={self.dist_wheels:.2f}m "
            f"odom_path={self.odom_path:.2f}m "
            f"disp={math.hypot(x - sx, y - sy):.2f}m "
            f"| x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg")

    def summary(self):
        self.f.flush()
        x, y = (self.last_xy or (0.0, 0.0))
        sx, sy = (self.start_xy or (x, y))
        print(f"\n=== {self.path} — {self.n} rows ===")
        print(f"  wheels say travelled   {self.dist_wheels:.3f} m")
        print(f"  odom pose path length  {self.odom_path:.3f} m")
        print(f"  net displacement       {math.hypot(x - sx, y - sy):.3f} m"
              f"   (from where logging started)")
        if self.dist_wheels > 0.05:
            ratio = self.odom_path / self.dist_wheels
            print(f"  odom_path / wheels     {ratio:.3f}")
            if ratio < 0.5:
                print("  -> wheels report motion the pose is NOT integrating")
            elif ratio > 1.5:
                print("  -> pose moves more than the wheels justify")
            else:
                print("  -> integration tracks the wheels; if the robot went "
                      "further than this, the WHEELS are under-reporting")


def main():
    rclpy.init()
    node = DriveLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.f.close()
        node.destroy_node()
        if rclpy.ok():          # Ctrl-C already shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
