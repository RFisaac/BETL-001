"""led_node: shows BETL's attention on the RP2350 LED ring.

  - One dot is always lit: the way BETL is "facing". It only moves when the wake word is heard,
    gliding to the speaker's direction (a stand-in for the robot turning its head).
  - Every other sound flashes a faint orange blip at its direction that fades out.
  - The dot's colour/animation follows the brain's state: idle, listening, thinking, speaking,
    stop (red pulse on the whole ring).

Subscribes: doa, level, wake_direction, state, speaking, speech_level
Parameters: see config/betl.yaml (port, LED 0 direction, reverse, noise thresholds, brightness)
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .ring_client import RingClient


class LedNode(Node):
    def __init__(self):
        super().__init__("led")
        p = self.declare_parameter
        p("port", "auto")
        p("num_leds", 16)
        p("led0_deg", 0.0)          # DoA angle (reSpeaker degrees) that LED 0 points at
        p("reverse", False)         # true if the ring counts the opposite way to the DoA
        p("home_deg", 0.0)          # where the dot rests at start-up
        p("brightness", 120)
        p("noise_db", -45.0)        # sounds quieter than this make no blip
        p("noise_full_db", -20.0)   # sounds this loud make a full-brightness blip
        p("noise_rate_hz", 12.0)
        p("mute_noise_while_speaking", True)
        g = lambda n: self.get_parameter(n).value

        self.ring = RingClient(g("port"), g("num_leds"), log=self.get_logger().info)
        self.doa = None
        self.state = "idle"
        self.speaking = False
        self.last_blip = 0.0
        self.last_level_sent = 0.0
        self.ready = False

        self.create_subscription(Float32, "doa", lambda m: setattr(self, "doa", m.data), 10)
        self.create_subscription(Float32, "level", self.on_level, 10)
        self.create_subscription(Float32, "wake_direction", self.on_wake_dir, 10)
        self.create_subscription(String, "state", self.on_state, 10)
        self.create_subscription(Bool, "speaking", self.on_speaking, 10)
        self.create_subscription(Float32, "speech_level", self.on_speech_level, 10)
        self.create_timer(1.0, self.keepalive)

    def pos(self, deg):
        n = self.get_parameter("num_leds").value
        rel = (deg - self.get_parameter("led0_deg").value) % 360
        if self.get_parameter("reverse").value:
            rel = (-rel) % 360
        return rel / 360.0 * n

    def setup(self):
        if self.ring.send(f"B {int(self.get_parameter('brightness').value)}"):
            self.ring.send(f"P {self.pos(self.get_parameter('home_deg').value):.2f}")
            self.ring.send(f"M {self.state}")
            self.ready = True

    def keepalive(self):
        if not self.ready:
            self.setup()

    def mode(self):
        return "speak" if self.speaking and self.state != "stop" else self.state

    def on_level(self, msg):
        if self.doa is None:
            return
        if self.speaking and self.get_parameter("mute_noise_while_speaking").value:
            return
        now = time.time()
        if now - self.last_blip < 1.0 / self.get_parameter("noise_rate_hz").value:
            return
        lo, hi = self.get_parameter("noise_db").value, self.get_parameter("noise_full_db").value
        if msg.data < lo:
            return
        s = min(1.0, (msg.data - lo) / max(1e-6, hi - lo))
        self.last_blip = now
        self.ring.send(f"N {self.pos(self.doa):.2f} {s:.2f}")

    def on_wake_dir(self, msg):
        self.ring.send(f"G {self.pos(msg.data):.2f}")

    def on_state(self, msg):
        self.state = msg.data
        self.ring.send(f"M {self.mode()}")

    def on_speaking(self, msg):
        self.speaking = msg.data
        self.ring.send(f"M {self.mode()}")

    def on_speech_level(self, msg):
        now = time.time()
        if now - self.last_level_sent > 0.04:
            self.last_level_sent = now
            self.ring.send(f"L {msg.data:.2f}")


def main():
    rclpy.init()
    node = LedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.ring.send("M off")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
