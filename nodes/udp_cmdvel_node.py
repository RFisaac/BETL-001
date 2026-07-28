#!/usr/bin/env python3
"""
udp_cmdvel_node.py — runs on the X5.

Listens for the Deck's UDP packets and republishes them as
geometry_msgs/Twist on /cmd_vel. Deliberately a separate node from the
ODESC driver so that /cmd_vel stays a normal ROS topic: the Foxglove
teleop panel (fallback), and later Nav2, publish to the same place and
go through the same safety layers in the driver.

Behavior:
  - deadman held  -> publish the commanded twist
  - deadman released -> publish zero twist
  - no packets for `link_timeout` -> publish zero once, then go quiet
    (the ODESC node's own watchdog is the second layer)

Run:
  source /opt/tros/humble/setup.bash
  python3 udp_cmdvel_node.py
"""

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class UdpCmdVel(Node):
    def __init__(self):
        super().__init__("udp_cmdvel")
        self.declare_parameter("port", 5005)
        self.declare_parameter("link_timeout", 0.5)

        port = self.get_parameter("port").value
        self.link_timeout = self.get_parameter("link_timeout").value

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/deck_status", 10)
        self.deadman = False
        self.create_timer(0.2, self.publish_status)  # 5 Hz for the HUD

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.last_rx = None
        self.sent_link_zero = True  # don't publish until first packet
        self.timer = self.create_timer(0.01, self.tick)  # 100 Hz poll
        self.get_logger().info(f"Listening on UDP :{port} -> /cmd_vel")

    def tick(self):
        newest = None
        while True:  # drain the socket, keep only the newest packet
            try:
                data, _ = self.sock.recvfrom(512)
                newest = data
            except BlockingIOError:
                break

        now = self.get_clock().now().nanoseconds * 1e-9

        if newest is not None:
            try:
                d = json.loads(newest.decode())
                self.deadman = bool(d.get("deadman", False))
                msg = Twist()
                if self.deadman:
                    msg.linear.x = float(d.get("vx", 0.0))
                    msg.angular.z = float(d.get("wz", 0.0))
                # deadman released -> zeros (msg default)
                self.pub.publish(msg)
                self.last_rx = now
                self.sent_link_zero = False
            except (ValueError, KeyError) as e:
                self.get_logger().warn(f"Bad packet: {e}")

        elif (self.last_rx is not None
              and now - self.last_rx > self.link_timeout
              and not self.sent_link_zero):
            self.pub.publish(Twist())  # one explicit zero on link loss
            self.sent_link_zero = True
            self.deadman = False
            self.get_logger().warn("Deck link lost — sent zero twist")

    def publish_status(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        link_ok = (self.last_rx is not None
                   and now - self.last_rx < self.link_timeout)
        msg = String()
        msg.data = json.dumps({"deadman": self.deadman and link_ok,
                               "link": link_ok})
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = UdpCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
