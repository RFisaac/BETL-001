#!/usr/bin/env python3
"""
hud_view_node.py — turns /cloud_3d into something a browser can actually hold.

    python3 /userdata/hud_view_node.py

WHY THIS EXISTS
  /cloud_3d at 200k points is 2.4 MB per message, ~3.2 MB base64'd through
  rosbridge. That is the same mistake as viewing raw camera frames: the cost
  is the wire, not the rendering. Filtering has to happen BEFORE the data
  crosses it.

  This subscribes to the full cloud, keeps only a horizontal slab, thins it
  to a point budget, and republishes a small JSON string. 4000 points lands
  around 45 KB — cheap enough to leave running for the whole demo.

WHY A Z-SLICE
  A top-down view of a whole room is mostly floor and ceiling, which are the
  two surfaces that tell you nothing. Cut a slab at roughly robot height and
  you get a floor plan: walls become lines, and a heading error shows up as
  the same wall appearing twice at an angle. That is the thing worth looking
  at while driving.

  z is in the `odom` frame, so z=0 is wherever camera_link started — roughly
  camera height, NOT the floor. Default slab -0.30..+0.60 m sits above the
  floor and below most ceilings. Both bounds are live:

      ros2 param set /hud_view z_min -0.5
      ros2 param set /hud_view z_max 0.2

  or from the console's LIDAR panel.

OUTPUT  /hud/map  (std_msgs/String, JSON)
  {"n":3812,"total":78450,"z":[-0.3,0.6],
   "bounds":[xmin,ymin,xmax,ymax],
   "pose":[x,y,yaw_deg],
   "pts":[[cx,cy,cz],...]}          centimetres, integers

  Integers in cm rather than floats: same useful precision at 5 cm voxels,
  roughly half the characters, and it stays readable under `ros2 topic echo`.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# /cloud_3d is published BEST_EFFORT. A RELIABLE subscriber will not match it
# and receives absolutely nothing, with only a QoS warning to show for it.
CLOUD_QOS = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)


class HudView(Node):
    def __init__(self):
        super().__init__("hud_view")

        self.declare_parameter("cloud_topic", "/cloud_3d")
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("out_topic", "/hud/map")
        # Slab, metres, in odom. Live — sweep them and watch the plan change.
        self.declare_parameter("z_min", -0.30)
        self.declare_parameter("z_max", 0.60)
        # Point budget. 4000 ~ 45 KB of JSON; 200k would be 2.4 MB of binary.
        self.declare_parameter("max_points", 4000)
        self.declare_parameter("publish_hz", 1.0)

        gp = lambda n: self.get_parameter(n).value
        self.max_points = int(gp("max_points"))

        self.latest = None          # newest cloud, decoded lazily
        self.pose = (0.0, 0.0, 0.0)
        self.n_in = 0
        self.n_out = 0

        self.create_subscription(
            PointCloud2, gp("cloud_topic"), self.on_cloud, CLOUD_QOS)
        self.create_subscription(
            Odometry, gp("odom_topic"), self.on_odom, 10)
        self.pub = self.create_publisher(String, gp("out_topic"), 1)

        hz = max(0.05, float(gp("publish_hz")))
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(10.0, self.log_health)
        self.get_logger().info(
            f"hud_view: {gp('cloud_topic')} -> {gp('out_topic')} "
            f"@ {hz} Hz, slab z {gp('z_min')}..{gp('z_max')} m, "
            f"budget {self.max_points} pts")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     math.degrees(yaw))

    def on_cloud(self, msg):
        # Store the raw message; decoding every arrival would waste work when
        # the cloud publishes faster than we do.
        self.latest = msg

    def tick(self):
        msg = self.latest
        if msg is None:
            return
        self.latest = None

        if msg.point_step != 12 or msg.width == 0:
            self.get_logger().warn(
                f"unexpected layout: point_step={msg.point_step}", once=True)
            return

        pts = np.frombuffer(bytes(msg.data), dtype=np.float32)
        pts = pts.reshape(-1, 3)
        self.n_in = pts.shape[0]

        z0 = float(self.get_parameter("z_min").value)
        z1 = float(self.get_parameter("z_max").value)
        keep = pts[(pts[:, 2] >= z0) & (pts[:, 2] <= z1)]

        if keep.shape[0] > self.max_points:
            # Even stride, not random: a stride preserves spatial structure so
            # walls stay continuous lines, where random sampling stipples them.
            step = int(np.ceil(keep.shape[0] / self.max_points))
            keep = keep[::step]
        self.n_out = keep.shape[0]

        if keep.shape[0]:
            cm = np.round(keep * 100.0).astype(np.int32)
            bounds = [float(keep[:, 0].min()), float(keep[:, 1].min()),
                      float(keep[:, 0].max()), float(keep[:, 1].max())]
            pts_out = cm.tolist()
        else:
            bounds = [0.0, 0.0, 0.0, 0.0]
            pts_out = []

        out = String()
        out.data = json.dumps({
            "n": self.n_out,
            "total": self.n_in,
            "z": [round(z0, 2), round(z1, 2)],
            "bounds": [round(b, 2) for b in bounds],
            "pose": [round(self.pose[0], 3), round(self.pose[1], 3),
                     round(self.pose[2], 1)],
            "pts": pts_out,
        }, separators=(",", ":"))
        self.pub.publish(out)

    def log_health(self):
        pct = (100.0 * self.n_out / self.n_in) if self.n_in else 0.0
        self.get_logger().info(
            f"slab {self.get_parameter('z_min').value:+.2f}.."
            f"{self.get_parameter('z_max').value:+.2f} m  "
            f"{self.n_out}/{self.n_in} pts ({pct:.0f}%)")


def main():
    rclpy.init()
    node = HudView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
