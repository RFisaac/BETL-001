#!/usr/bin/env python3
"""
wheel_odom_node.py — dead-reckoning pose from wheel encoders + gyro yaw.

Publishes odom -> camera_link so the lidar accumulator has a pose source
that does not depend on visual odometry.

WHY THIS SPLIT
  Wheels measure distance well and rotation badly — skid-steer scrubs
  sideways through every turn, so differential wheel speed over-reports
  yaw by an unknown amount. A gyro is the exact opposite: excellent
  short-term rotation, useless for position. So take forward velocity
  from the wheels and yaw rate from the gyro.

  This is NOT the IMU usage that failed earlier. That was RTAB-Map
  trying to pair IMU samples with camera frames whose timestamps sit
  ~230 ms off. Here the gyro is read directly for angular rate — no
  pairing with images, so no sync problem to have.

GYRO BIAS
  A stationary gyro never reads exactly zero, and integrating that bias
  is pure heading drift. This tracks bias continuously using the wheels
  as a zero-motion reference: whenever the wheels report stopped, the
  gyro reading *is* the bias, so it gets averaged in. Handles warm-up
  drift for free. (Classic zero-velocity update.)

AXIS — CHECK THIS FIRST
  Which gyro axis is yaw depends on how the camera is mounted. The
  RealSense optical frame is x-right, y-down, z-forward, so for a
  forward-looking camera robot yaw is usually about -y. Verify before
  trusting it:

      ros2 topic echo /camera/camera/imu --field angular_velocity

  Rotate the robot left and right by hand. Whichever component swings
  hardest is the yaw axis; if it reads negative while turning left, set
  gyro_sign to -1.

CALIBRATION SLOTS (both no-ops at their defaults)
  gyro_scale  1.0    filled from the turntable run. Multiplies the
                     bias-corrected yaw rate. 1-3% off is normal.
  learn_bias  true   set FALSE on any fixture where the robot rotates
                     with the wheels stopped (turntable, hand-pushed).
                     Otherwise the ZUPT averages that rotation into bias.

Run (X5) — every value below is already the default, so just:
    python3 /userdata/wheel_odom_node.py

    # If you want to be explicit, gyro_axis MUST be quoted. Bare `y` is
    # boolean true in YAML 1.1, and rcl's parser follows that, so
    # `gyro_axis:=y` arrives as True and throws
    # InvalidParameterTypeException before the node starts. `x` and `z`
    # are unaffected — y is the only axis letter YAML steals.
    python3 /userdata/wheel_odom_node.py --ros-args \
        -p use_imu_yaw:=true -p "gyro_axis:='y'" -p gyro_sign:=-1

    # after calibration (the decimal point is required — `:=1` is an int
    # and throws against a double parameter):
    #   -p gyro_scale:=1.0234
    # on the turntable:
    #   -p learn_bias:=false      (or: ros2 param set /wheel_odom learn_bias false)

    # wheels only, no gyro:
    python3 /userdata/wheel_odom_node.py --ros-args -p use_imu_yaw:=false

NOT SLAM: dead reckoning only. No loop closure, no global correction.
Good for minutes of gentle driving, not hours.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class WheelOdom(Node):
    def __init__(self):
        super().__init__("wheel_odom")

        self.declare_parameter("wheel_states_topic", "/wheel_states")
        self.declare_parameter("imu_topic", "/camera/camera/imu")
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("odom_frame", "odom")
        # camera_link keeps the existing chain intact:
        #   odom -> camera_link -> stage_base -> stage_mount -> laser
        self.declare_parameter("base_frame", "camera_link")

        self.declare_parameter("wheel_radius", 0.0765)
        self.declare_parameter("track_width", 0.375)
        # Only used when the gyro is off. <1.0 is normal for skid steer.
        self.declare_parameter("scrub_factor", 1.0)
        # ODESCs report ~0.045 turns/s (~0.28 rad/s) of observer noise at
        # standstill. Without a deadband the robot "drives" while parked.
        self.declare_parameter("velocity_deadband", 0.35)     # rad/s
        # How many of the four wheels must exceed the deadband before any
        # motion is believed. 3 rejects one- and two-wheel noise blips.
        self.declare_parameter("min_wheels_moving", 2)
        # Rotation rate, rad/s, below which a wheels-stopped robot is treated
        # as genuinely still. "Wheels stopped => not rotating" is only a proxy,
        # and it is false on a turntable or when the robot is nudged; without
        # this the heading silently stops tracking. Gyro noise and bias
        # residual sit around 0.001-0.02 rad/s, a deliberate turn is 0.5-2,
        # so 0.05 (2.9 deg/s) separates them with room either side.
        self.declare_parameter("parked_w_max", 0.05)

        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("gyro_axis", "y")              # x | y | z
        self.declare_parameter("gyro_sign", -1)
        # CALIBRATION SLOT — 1.0 is a no-op until the turntable run fills it.
        # Multiplies the bias-corrected rate, so it corrects both true gyro
        # scale error (1-3% typical for MEMS) and the cosine of any camera
        # mounting tilt. Those two are inseparable by this measurement and
        # you want them folded into one number anyway.
        # Read live so `ros2 param set` tunes it without a restart.
        self.declare_parameter("gyro_scale", 1.0)
        # "Wheels stopped => the robot is not moving." True on the floor,
        # FALSE on a turntable or when rotated by hand. Clearing this does two
        # things, both needed for that case:
        #   1. stops the ZUPT averaging a real rotation into the gyro bias
        #   2. stops yaw being pinned to zero while the wheels read stopped
        # The console's yaw calibration clears and restores it automatically.
        self.declare_parameter("learn_bias", True)
        self.declare_parameter("gyro_bias_alpha", 0.001)      # EMA when parked
        self.declare_parameter("imu_timeout", 0.5)            # s
        self.declare_parameter("publish_tf", True)

        gp = lambda n: self.get_parameter(n).value
        self.odom_frame = gp("odom_frame")
        self.base_frame = gp("base_frame")
        self.r = float(gp("wheel_radius"))
        self.track = float(gp("track_width"))
        self.scrub = float(gp("scrub_factor"))
        self.deadband = float(gp("velocity_deadband"))
        self.min_moving = int(gp("min_wheels_moving"))
        self.use_imu = bool(gp("use_imu_yaw"))
        self.axis = str(gp("gyro_axis")).lower()
        self.gsign = float(gp("gyro_sign"))
        self.alpha = float(gp("gyro_bias_alpha"))
        self.imu_timeout = float(gp("imu_timeout"))
        self.do_tf = bool(gp("publish_tf"))

        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.parked = True
        self.gyro_raw = 0.0
        self.gyro_bias = 0.0
        self.gyro_stamp = None
        self.n_bias = 0
        self.n_msgs = 0
        self.using_gyro = False

        self.create_subscription(
            JointState, gp("wheel_states_topic"), self.on_wheels, 20)
        if self.use_imu:
            # RealSense publishes IMU with sensor-data QoS (BEST_EFFORT).
            # A RELIABLE subscriber will NOT match a BEST_EFFORT publisher —
            # DDS refuses the connection and delivers zero messages, with
            # only a QoS warning at startup to show for it. The node then
            # falls back to scrub-corrupted wheels-only yaw and looks like
            # it is working. Declared explicitly here so that can't recur.
            # BEST_EFFORT on the subscriber side matches both, so this is
            # strictly more permissive than the default.
            imu_qos = QoSProfile(depth=50,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Imu, gp("imu_topic"), self.on_imu,
                                     imu_qos)
        self.pub = self.create_publisher(Odometry, gp("odom_topic"), 10)
        # Bias is the number the whole heading estimate rests on, and nothing
        # exposed it — a corrupted one looks identical to a good one until the
        # map comes out twisted. Published so the console can show it.
        self.status_pub = self.create_publisher(String, "/wheel_odom_status", 1)
        self.create_timer(1.0, self.publish_status)
        self.create_service(Trigger, "~/reset_bias", self.on_reset_bias)
        self.tf_bc = TransformBroadcaster(self)
        self.create_timer(5.0, self.log_health)

        src = (f"gyro {self.gsign:+.0f}*{self.axis}" if self.use_imu
               else f"wheels (scrub={self.scrub})")
        self.get_logger().info(
            f"wheel odometry: {self.odom_frame} -> {self.base_frame}, "
            f"r={self.r} m, track={self.track} m, yaw from {src}")

    # ---------- gyro ----------

    def on_imu(self, msg):
        g = msg.angular_velocity
        raw = {"x": g.x, "y": g.y, "z": g.z}.get(self.axis, g.z)
        self.gyro_raw = self.gsign * raw
        self.gyro_stamp = self.get_clock().now()

        # Wheels stopped => whatever the gyro reads is bias. Two guards:
        #   learn_bias   explicit off-switch, used during calibration
        #   quiet check  reject samples far from the current estimate, so a
        #                real rotation cannot be averaged in even if someone
        #                forgets to switch learning off
        # The n_bias < 200 bootstrap matters: after reset_bias the estimate is
        # 0 and every sample looks like an outlier, so the gate has to stay
        # open until it has something to compare against.
        quiet = (self.n_bias < 200 or
                 abs(self.gyro_raw - self.gyro_bias) <
                 float(self.get_parameter("parked_w_max").value))
        if self.parked and quiet and self.get_parameter("learn_bias").value:
            a = 1.0 / min(self.n_bias + 1, 200) if self.n_bias < 200 \
                else self.alpha
            self.gyro_bias += a * (self.gyro_raw - self.gyro_bias)
            self.n_bias += 1

    def gyro_yaw_rate(self):
        """Bias-corrected yaw rate, or None if the gyro isn't usable."""
        if not self.use_imu or self.gyro_stamp is None or self.n_bias < 50:
            return None
        age = (self.get_clock().now() - self.gyro_stamp).nanoseconds * 1e-9
        if age > self.imu_timeout:
            return None
        # Order matters: subtract bias FIRST, then scale. Bias is an additive
        # offset in raw sensor units, so it stays valid when gyro_scale is
        # retuned. Scaling first would bake the current scale into every
        # stored bias value and invalidate it on the next calibration.
        return (self.gyro_raw - self.gyro_bias) * \
            float(self.get_parameter("gyro_scale").value)

    # ---------- wheels ----------

    def on_wheels(self, msg):
        if not msg.name or not msg.velocity:
            return

        left, right = [], []
        n_moving = 0
        for name, vel in zip(msg.name, msg.velocity):
            v = 0.0 if abs(vel) < self.deadband else vel
            if v != 0.0:
                n_moving += 1
            if "left" in name:
                left.append(v)
            elif "right" in name:
                right.append(v)
        if not left or not right:
            return

        # A real drive turns every wheel. A noise blip turns one. Measured
        # 2026-07-28: 0.202 m of phantom distance accrued over one 137 s
        # run purely from single wheels crossing the deadband while the
        # robot stood still — ~5% of a 4 m drive, and it grows with TIME
        # rather than distance, so idling between legs is the worst case.
        # Requiring a quorum kills it without raising the deadband, which
        # would otherwise clip genuinely slow motion on the accel ramps.
        if n_moving < self.min_moving:
            left = [0.0] * len(left)
            right = [0.0] * len(right)

        # rad/s at the wheel -> m/s at the contact patch
        vl = (sum(left) / len(left)) * self.r
        vr = (sum(right) / len(right)) * self.r
        self.parked = (abs(vl) < 1e-6 and abs(vr) < 1e-6)

        v = 0.5 * (vl + vr)                          # forward, m/s

        w_gyro = self.gyro_yaw_rate()
        if w_gyro is not None:
            w = w_gyro                               # gyro wins on rotation
            self.using_gyro = True
        else:
            w = self.scrub * (vr - vl) / self.track  # fallback
            self.using_gyro = False

        t = Time.from_msg(msg.header.stamp)
        if self.last_t is None:
            self.last_t = t
            return
        dt = (t - self.last_t).nanoseconds * 1e-9
        self.last_t = t
        if dt <= 0.0 or dt > 1.0:      # first tick, or a stall — skip
            return

        # Parked AND quiet => genuinely still, so do not let noise rotate the
        # map. Parked but turning fast => something is rotating the robot
        # without the wheels, and that is real motion worth tracking. This
        # used to be gated on learn_bias alone, which meant a turntable only
        # worked during calibration.
        if self.parked and w_gyro is not None and \
                abs(w_gyro) < float(self.get_parameter("parked_w_max").value):
            w = 0.0

        # midpoint integration: better through turns than naive Euler
        th_mid = self.th + 0.5 * w * dt
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.th = math.atan2(math.sin(self.th + w * dt),
                             math.cos(self.th + w * dt))

        self.n_msgs += 1
        self.publish(msg.header.stamp, v, w)

    # ---------- output ----------

    def publish(self, stamp, v, w):
        qz = math.sin(self.th * 0.5)
        qw = math.cos(self.th * 0.5)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        # Dead reckoning: honest, generous covariance. Yaw worst.
        od.pose.covariance[0] = od.pose.covariance[7] = 0.05
        od.pose.covariance[35] = 0.1 if self.using_gyro else 0.3
        self.pub.publish(od)

        if self.do_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "bias": round(self.gyro_bias, 6),
            "n_bias": self.n_bias,
            "yaw_src": "gyro" if self.using_gyro else "wheels",
            "parked": bool(self.parked),
            "learn_bias": bool(self.get_parameter("learn_bias").value),
            "scale": float(self.get_parameter("gyro_scale").value),
            "yaw_deg": round(math.degrees(self.th), 2),
        })
        self.status_pub.publish(msg)

    def on_reset_bias(self, request, response):
        """Throw the bias estimate away and relearn from stationary.

        Needed because a hand rotation while learn_bias was on averages the
        rotation INTO the bias, and there is otherwise no way back short of
        restarting the node. Leave the robot still for ~10 s afterwards."""
        old = self.gyro_bias
        self.gyro_bias = 0.0
        self.n_bias = 0
        response.success = True
        response.message = (f"bias {old:+.5f} -> 0, relearning; "
                            f"keep the robot still ~10 s")
        self.get_logger().warn(response.message)
        return response

    def log_health(self):
        src = "GYRO" if self.using_gyro else "wheels"
        scale = float(self.get_parameter("gyro_scale").value)
        extra = "" if scale == 1.0 else f" scale={scale:.4f}"
        if not self.get_parameter("learn_bias").value:
            extra += " bias_LOCKED"
        self.get_logger().info(
            f"msgs={self.n_msgs} x={self.x:.2f} y={self.y:.2f} "
            f"yaw={math.degrees(self.th):.1f}deg  yaw_src={src} "
            f"bias={self.gyro_bias:+.5f} rad/s (n={self.n_bias}) "
            f"{'parked' if self.parked else 'MOVING'}{extra}")


def main():
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
