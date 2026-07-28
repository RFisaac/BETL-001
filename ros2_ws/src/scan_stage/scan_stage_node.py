#!/usr/bin/env python3
"""
scan_stage_node -- ingest the rotating-lidar stage Pico's USB serial stream and
publish stage angle as a JointState + a live TF, so the RPLIDAR's moving scan plane
can be reprojected into 3D.

Serial protocol (from scan_stage.py firmware), newline-terminated ASCII:
  Pico -> X5:
    A <pico_us> <accum_count>   high-rate angle sample (motor-shaft counts, unwrapped)
    SYNC <pico_us>              clock reference (boot + on request)
    STAT <pico_us> <rpm> <duty> <agc> <mag> <det>
    EVT <pico_us> <text>
  X5 -> Pico:
    spin / stop / sync / stream on|off

Time domain:
  Each angle sample was timestamped ON THE PICO at read time (pico_us). The Pico
  clock and ROS clock differ; we estimate a fixed offset as the running MINIMUM of
  (ros_now - pico_us) across samples -- the minimum-latency sample is the one that
  spent least time buffered, so it gives the cleanest offset. We then stamp each
  published sample with pico_us + offset, recovering measurement time independent
  of USB/serial jitter.

Angle:
  Encoder is on the MOTOR shaft; a GT2 120:24 = 5:1 belt reduces to the stage.
  stage_angle_rad = accum_count * (2*pi) / (COUNTS_PER_MOTOR_REV * GEAR_RATIO)
  Gear ratio and sign live HERE (one place), not in firmware, so they're correctable
  without reflashing.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster

import serial  # pyserial


COUNTS_PER_MOTOR_REV = 4096
GEAR_RATIO = 5.0                      # 120T stage / 24T motor
COUNTS_PER_STAGE_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO   # 20480


class ScanStageNode(Node):
    def __init__(self):
        super().__init__('scan_stage_node')

        # ---- parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)          # CDC-ACM ignores baud, harmless
        self.declare_parameter('base_frame', 'stage_base')
        self.declare_parameter('lidar_frame', 'laser')
        self.declare_parameter('joint_name', 'stage_rotation')
        self.declare_parameter('angle_sign', 1)          # flip to -1 if TF spins backwards
        # Stage zero, degrees. Live: read on every sample, so the console can
        # sweep it while the stage spins and you watch the cloud square up.
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('counts_per_stage_rev', COUNTS_PER_STAGE_REV)
        # rotation axis of the stage in base_frame; default spins about Z
        self.declare_parameter('axis', 'z')              # 'x' | 'y' | 'z'
        # offset estimator: reset the running-min every N seconds so slow clock drift
        # between the two oscillators is tracked rather than frozen forever.
        self.declare_parameter('offset_reset_sec', 30.0)
        # duty used when /stage/spin is called (signed: + fwd, - rev, 0..65535 magnitude)
        self.declare_parameter('spin_duty', 40000)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.joint_name = self.get_parameter('joint_name').value
        self.angle_sign = int(self.get_parameter('angle_sign').value)
        self.counts_per_rev = float(self.get_parameter('counts_per_stage_rev').value)
        self.axis = self.get_parameter('axis').value
        self.offset_reset_sec = float(self.get_parameter('offset_reset_sec').value)

        # ---- clock offset estimator state ----
        # offset = ros_time_ns - pico_us*1000 ; we keep the running minimum.
        self._offset_ns = None
        self._offset_window_start_ns = None

        # ---- publishers ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.joint_pub = self.create_publisher(JointState, 'stage_angle', sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- services to command the Pico ----
        self.spin_srv = self.create_service(Trigger, 'stage/spin', self._on_spin)
        self.stop_srv = self.create_service(Trigger, 'stage/stop', self._on_stop)

        # ---- live speed/direction: publish an Int32 signed duty to /stage/set_duty ----
        #   positive = forward, negative = reverse, magnitude 0..65535
        self.duty_sub = self.create_subscription(
            Int32, 'stage/set_duty', self._on_set_duty, 10)

        # ---- serial ----
        self.get_logger().info(f'Opening {self.port} @ {self.baud}')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open {self.port}: {e}')
            raise

        self._ser_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # telemetry cache
        self.last_rpm = 0.0
        self.last_det = 0

        self.get_logger().info(
            f'scan_stage_node up. counts/stage_rev={self.counts_per_rev:.0f}, '
            f'axis={self.axis}, sign={self.angle_sign}, '
            f'{self.base_frame} -> {self.lidar_frame}')

    # ---------------- serial reader thread ----------------
    def _read_loop(self):
        buf = b''
        while not self._stop_reader.is_set():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'serial read error: {e}')
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle_line(line.decode('ascii', errors='ignore').strip())

    def _handle_line(self, line):
        if not line:
            return
        parts = line.split()
        tag = parts[0]

        if tag == 'A' and len(parts) == 3:
            try:
                pico_us = int(parts[1])
                count = int(parts[2])
            except ValueError:
                return
            self._on_angle(pico_us, count)

        elif tag == 'STAT' and len(parts) >= 7:
            try:
                self.last_rpm = float(parts[2])
                self.last_det = int(parts[6])
            except ValueError:
                pass
            if self.last_det == 0:
                self.get_logger().warn('AS5600 reports magnet NOT detected', throttle_duration_sec=5.0)

        elif tag == 'SYNC' and len(parts) == 2:
            # seed/refresh the offset window on an explicit sync
            self._reset_offset_window()

        elif tag == 'EVT':
            self.get_logger().info(f'Pico EVT: {line}')

    # ---------------- clock offset ----------------
    def _reset_offset_window(self):
        self._offset_ns = None
        self._offset_window_start_ns = None

    def _corrected_stamp_ns(self, pico_us):
        """Return ROS-time ns for a Pico micros() reading, via running-min offset."""
        ros_now_ns = self.get_clock().now().nanoseconds
        pico_ns = pico_us * 1000
        sample_offset = ros_now_ns - pico_ns

        if self._offset_window_start_ns is None:
            self._offset_window_start_ns = ros_now_ns

        # periodically reset so slow relative drift is re-estimated
        if (ros_now_ns - self._offset_window_start_ns) > self.offset_reset_sec * 1e9:
            self._offset_ns = None
            self._offset_window_start_ns = ros_now_ns

        if self._offset_ns is None or sample_offset < self._offset_ns:
            self._offset_ns = sample_offset

        return pico_ns + self._offset_ns

    # ---------------- angle -> messages ----------------
    def _on_angle(self, pico_us, count):
        stamp_ns = self._corrected_stamp_ns(pico_us)

        angle_rad = self.angle_sign * count * (2.0 * math.pi) / self.counts_per_rev

        # Stage zero. Re-read every sample so it tunes live from the console —
        # this is where the zero BELONGS, because it is an encoder offset, not
        # mounting geometry. It used to live in the `--yaw` of the
        # camera_link -> stage_base static publisher, which cannot be changed
        # without restarting that publisher.
        angle_rad += math.radians(
            float(self.get_parameter('angle_offset_deg').value))

        # normalize into (-pi, pi] for the joint position (TF quaternion is fine either way)
        wrapped = math.atan2(math.sin(angle_rad), math.cos(angle_rad))

        stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = [self.joint_name]
        js.position = [wrapped]
        js.velocity = [self.last_rpm / 60.0 * 2.0 * math.pi / GEAR_RATIO]  # stage rad/s from motor rpm
        self.joint_pub.publish(js)

        # ---- TF: base -> lidar, rotation about the chosen axis ----
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        half = angle_rad / 2.0
        s = math.sin(half)
        if self.axis == 'x':
            t.transform.rotation.x = s
            t.transform.rotation.w = math.cos(half)
        elif self.axis == 'y':
            t.transform.rotation.y = s
            t.transform.rotation.w = math.cos(half)
        else:  # z
            t.transform.rotation.z = s
            t.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform(t)

    # ---------------- services ----------------
    def _send(self, text):
        with self._ser_lock:
            self.ser.write((text + '\n').encode('ascii'))

    def _on_spin(self, request, response):
        duty = int(self.get_parameter('spin_duty').value)
        duty = max(-65535, min(65535, duty))
        self._send('spin {}'.format(duty))
        response.success = True
        response.message = 'spin {} sent'.format(duty)
        return response

    def _on_stop(self, request, response):
        self._send('stop')
        response.success = True
        response.message = 'stop sent'
        return response

    def _on_set_duty(self, msg):
        duty = max(-65535, min(65535, int(msg.data)))
        if duty == 0:
            self._send('stop')
        else:
            self._send('spin {}'.format(duty))

    def destroy_node(self):
        self._stop_reader.set()
        try:
            self._send('stop')
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanStageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
