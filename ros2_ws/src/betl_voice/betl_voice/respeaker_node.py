"""respeaker_node: owns the reSpeaker's USB control interface.

Publishes (in the node's namespace, 'betl' by default):
  doa             std_msgs/Float32   direction of the loudest source, degrees 0-359, ~20 Hz
  voice_activity  std_msgs/Bool      the XVF3800's own speech detector
  wake_direction  std_msgs/Float32   where the person who said the wake word is (once per wake)
Subscribes:
  wake            std_msgs/String    from listener_node; triggers a wake_direction estimate

wake_direction is the circular mean of the DoA readings taken while speech was detected in
the second or so before the wake word finished, so it points at the speaker, not at the last
random noise. Anything that should turn the robot toward the user can subscribe to it.
"""

import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .xvf import XVF, circular_mean_deg


class RespeakerNode(Node):
    def __init__(self):
        super().__init__("respeaker")
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("wake_window_s", 1.5)
        self.declare_parameter("usb_pid", 0)

        self.pub_doa = self.create_publisher(Float32, "doa", 10)
        self.pub_vad = self.create_publisher(Bool, "voice_activity", 10)
        self.pub_wake_dir = self.create_publisher(Float32, "wake_direction", 10)
        self.create_subscription(String, "wake", self.on_wake, 10)

        self.history = deque()           # (time, doa, speech)
        self.lock = threading.Lock()
        self.xvf = None
        self.last_vad = None
        self.warned = False
        period = 1.0 / self.get_parameter("poll_hz").value
        self.create_timer(period, self.poll)

    def poll(self):
        if self.xvf is None:
            try:
                pid = self.get_parameter("usb_pid").value or None
                self.xvf = XVF(pid=pid)
                ver = ".".join(str(v) for v in self.xvf.read("VERSION"))
                self.get_logger().info(f"reSpeaker control connected, firmware {ver}")
                self.warned = False
            except Exception as e:
                if not self.warned:
                    self.get_logger().warn(f"reSpeaker control not available ({e}); retrying. "
                                           "On Linux check the udev rule in udev/99-betl.rules.")
                    self.warned = True
                self.xvf = None
                return
        try:
            doa, speech = self.xvf.doa()
        except Exception as e:
            self.get_logger().warn(f"lost reSpeaker control ({e}); reconnecting")
            self.xvf.close()
            self.xvf = None
            return
        now = time.time()
        with self.lock:
            self.history.append((now, doa, speech))
            while self.history and now - self.history[0][0] > 10:
                self.history.popleft()
        self.pub_doa.publish(Float32(data=float(doa)))
        if speech != self.last_vad:
            self.pub_vad.publish(Bool(data=speech))
            self.last_vad = speech

    def on_wake(self, msg):
        window = self.get_parameter("wake_window_s").value
        now = time.time()
        with self.lock:
            recent = [(d, s) for (t, d, s) in self.history if now - t <= window]
        speech_doas = [d for d, s in recent if s]
        direction = circular_mean_deg(speech_doas) if speech_doas else (
            recent[-1][0] if recent else None)
        if direction is None:
            self.get_logger().warn("wake word heard but no direction data yet")
            return
        self.pub_wake_dir.publish(Float32(data=float(direction)))
        self.get_logger().info(f"wake '{msg.data}' from {direction:.0f} deg "
                               f"({len(speech_doas)} speech samples)")


def main():
    rclpy.init()
    node = RespeakerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.xvf:
            node.xvf.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
