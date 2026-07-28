#!/usr/bin/env python3
"""
scan_accumulator_node.py — the missing piece of the spherical scanner.

Subscribes /scan (LaserScan from the canted A1), transforms every point
into a fixed frame using the live TF chain, and appends it into a
growing voxel-downsampled PointCloud2 published on /cloud_3d.

Because the TF chain is
    map -> odom -> camera_link -> stage_base -> stage_mount -> laser
a single TF lookup at a point's own timestamp corrects for BOTH the
robot's motion AND the stage's rotation at once. That's the whole trick:
you don't deskew the stage and the base separately, tf2 does both.

Deskewing is CHUNKED, not per-point: each scan is split into `segments`
angular slices with one TF lookup each. At 16 segments and 5.5 Hz that's
~88 lookups/sec, which the A55s handle fine, and it captures nearly all
of the correction.

SELF-FILTER: points falling inside an axis-aligned box in `mask_frame`
are dropped, so the robot's own body doesn't accumulate into the map.
All six box bounds are live-tunable with `ros2 param set` — no restart.

TUNING THE BOX: set publish_body:=true and open /cloud_body in Foxglove.
That cloud accumulates in the BODY frame, so self-hits pile up as a
sharp solid shape (your robot) while world geometry smears into haze as
you drive. Read the extents off the solid part, type them in, watch the
box vanish from /cloud_3d live.

Run (X5):
    source /opt/tros/humble/setup.bash
    python3 scan_accumulator_node.py --ros-args \
        -p fixed_frame:=odom -p voxel_size:=0.05 \
        -p publish_hz:=1.0 -p publish_body:=true

Ctrl-C writes the accumulated cloud to a .pcd in the CWD.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Trigger


def quat_to_mat(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix. No tf_transformations dependency."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ])


def transform_to_mat(t):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    q = t.transform.rotation
    v = t.transform.translation
    m = np.eye(4)
    m[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    m[:3, 3] = (v.x, v.y, v.z)
    return m


def invert_mat(m):
    """Fast inverse of a rigid 4x4 (transpose rotation, rotate translation)."""
    out = np.eye(4)
    r = m[:3, :3]
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ m[:3, 3]
    return out


def make_cloud_msg(pts, frame_id, stamp):
    """Nx3 float array -> PointCloud2. Built by hand so there's no
    sensor_msgs_py dependency to go missing at 2am."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = pts.astype(np.float32).tobytes()
    return msg


class ScanAccumulator(Node):
    def __init__(self):
        super().__init__("scan_accumulator")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/cloud_3d")
        self.declare_parameter("body_topic", "/cloud_body")
        # odom = smooth, no loop-closure jumps rewriting history.
        # map  = globally consistent, but already-placed points don't move
        #        when a loop closure fires, so old geometry can misalign.
        self.declare_parameter("fixed_frame", "odom")
        # The chain is split at base_frame:
        #   fixed_frame <- base_frame   SLOW link (odometry: laggy, jittery)
        #   base_frame  <- laser        FAST link (stage encoder at 150 Hz)
        # Querying both at one timestamp forces the fast link down to the
        # slow link's staleness, and odometry delay JITTERS (0.41-0.67 s),
        # so each scan would get the stage angle from a random moment in
        # the past. That scrambles the sphere. Looked up separately, the
        # stage stays accurate and only the pose is stale.
        self.declare_parameter("base_frame", "camera_link")
        self.declare_parameter("voxel_size", 0.05)      # m
        self.declare_parameter("segments", 16)          # deskew chunks per scan
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("range_min", 0.15)       # m
        self.declare_parameter("range_max", 12.0)       # m
        self.declare_parameter("max_voxels", 200000)
        self.declare_parameter("tf_timeout", 0.05)      # s

        # ---- self-filter: axis-aligned box in mask_frame, LIVE-TUNABLE ----
        # stage_base is rigid to the robot body and is the lidar's parent,
        # so the box doesn't depend on the RealSense pose at all.
        self.declare_parameter("mask_frame", "stage_base")
        self.declare_parameter("mask_enable", True)
        self.declare_parameter("mask_min_x", -0.60)
        self.declare_parameter("mask_max_x", 0.25)
        self.declare_parameter("mask_min_y", -0.30)
        self.declare_parameter("mask_max_y", 0.30)
        self.declare_parameter("mask_min_z", -1.00)
        self.declare_parameter("mask_max_z", 0.05)

        # ---- body-frame diagnostic cloud (for tuning the box by eye) ----
        self.declare_parameter("publish_body", False)
        self.declare_parameter("body_voxel", 0.03)
        self.declare_parameter("body_max_voxels", 150000)

        gp = lambda n: self.get_parameter(n).value
        self.fixed_frame = gp("fixed_frame")
        self.base_frame = gp("base_frame")
        self.mask_frame = gp("mask_frame")
        self.voxel = float(gp("voxel_size"))
        self.segments = int(gp("segments"))
        self.range_min = float(gp("range_min"))
        self.range_max = float(gp("range_max"))
        self.max_voxels = int(gp("max_voxels"))
        self.tf_timeout = Duration(seconds=float(gp("tf_timeout")))
        self.body_voxel = float(gp("body_voxel"))
        self.body_max = int(gp("body_max_voxels"))
        self.publish_body = bool(gp("publish_body"))

        self.grid = {}          # world-frame voxels
        self.body = {}          # body-frame voxels (diagnostic)
        self.n_scans = 0
        self.n_dropped = 0
        self.n_degraded = 0     # fell back to latest TF instead of stamped
        self.n_masked = 0       # points rejected as self-hits
        self.n_kept = 0
        self.capped = False
        # What T_fix_base actually contained on the most recent scan.
        # Logged rather than inferred: a pose that is frozen, or that has
        # rotation but no translation, is otherwise invisible from outside.
        self._pose_xyz = None
        self._pose_yaw = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True is REQUIRED: lookup_transform with a timeout blocks
        # the calling thread, and on the default single-threaded executor that
        # is the same thread that receives /tf — so the buffer could never fill
        # while a lookup waited on it. Give the listener its own thread.
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True)
        self._last_tf_error = None

        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            LaserScan, gp("scan_topic"), self.on_scan, qos)
        self.pub = self.create_publisher(PointCloud2, gp("cloud_topic"), 1)
        self.body_pub = self.create_publisher(PointCloud2, gp("body_topic"), 1)

        self.create_timer(1.0 / float(gp("publish_hz")), self.publish_clouds)
        self.create_timer(5.0, self.log_health)
        self.create_service(Trigger, "~/save", self.on_save)
        self.create_service(Trigger, "~/clear", self.on_clear)

        self.get_logger().info(
            f"accumulating {gp('scan_topic')} -> {gp('cloud_topic')} in "
            f"'{self.fixed_frame}', voxel={self.voxel} m, "
            f"{self.segments} deskew segments, "
            f"self-filter {'ON' if gp('mask_enable') else 'OFF'} "
            f"in '{self.mask_frame}'")

    # ---------- helpers ----------

    def mask_box(self):
        """Re-read every scan so `ros2 param set` tunes it live."""
        g = self.get_parameter
        return (bool(g("mask_enable").value),
                float(g("mask_min_x").value), float(g("mask_max_x").value),
                float(g("mask_min_y").value), float(g("mask_max_y").value),
                float(g("mask_min_z").value), float(g("mask_max_z").value))

    def lookup_stamped(self, target, source, stamp):
        """Exact-time lookup for the FAST link. Falls back to latest, but
        that fallback is what scrambles the cloud, so it's counted."""
        try:
            return self.tf_buffer.lookup_transform(
                target, source, stamp, self.tf_timeout), False
        except Exception as e:
            self._last_tf_error = f"stamped {target}<-{source}: {e}"
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time()), True
        except Exception as e:
            self._last_tf_error = f"latest {target}<-{source}: {e}"
            return None, False

    def lookup_latest(self, target, source):
        """Latest-available lookup for the SLOW link. Staleness here only
        shifts the cloud slightly, because the robot moves slowly."""
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except Exception as e:
            self._last_tf_error = f"pose {target}<-{source}: {e}"
            return None

    # ---------- core ----------

    def on_scan(self, scan):
        n = len(scan.ranges)
        if n == 0:
            return

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        valid = (np.isfinite(ranges) &
                 (ranges > max(self.range_min, scan.range_min)) &
                 (ranges < min(self.range_max, scan.range_max)))
        if not valid.any():
            return

        # per-sample dt: prefer the driver's time_increment, else derive it
        dt = scan.time_increment
        if not dt or dt <= 0.0:
            dt = (scan.scan_time / n) if scan.scan_time > 0 else 0.0

        base_t = Time.from_msg(scan.header.stamp)

        # --- SLOW link: robot pose, latest available. ---
        if self.fixed_frame == self.base_frame:
            T_fix_base = np.eye(4)
        else:
            tf_pose = self.lookup_latest(self.fixed_frame, self.base_frame)
            if tf_pose is None:
                self.n_dropped += 1
                return
            T_fix_base = transform_to_mat(tf_pose)

        # Record what we are ACTUALLY about to apply. A successful lookup
        # only proves the transform existed, not that it is changing.
        self._pose_xyz = tuple(T_fix_base[:3, 3])
        self._pose_yaw = math.degrees(
            math.atan2(T_fix_base[1, 0], T_fix_base[0, 0]))

        enabled, x0, x1, y0, y1, z0, z1 = self.mask_box()
        seg_bounds = np.linspace(0, n, self.segments + 1).astype(int)

        world_keep, body_all = [], []
        degraded_any = False

        for s in range(self.segments):
            lo, hi = seg_bounds[s], seg_bounds[s + 1]
            if hi <= lo:
                continue
            m = valid[lo:hi]
            if not m.any():
                continue

            mid = (lo + hi) // 2
            stamp = base_t + Duration(seconds=mid * dt) if dt > 0 else base_t

            # --- FAST link: stage angle at the scan's real timestamp. ---
            tf_bl, degraded = self.lookup_stamped(
                self.base_frame, scan.header.frame_id, stamp)
            if tf_bl is None:
                continue
            degraded_any |= degraded

            r = ranges[lo:hi][m]
            a = angles[lo:hi][m]
            # A1 scans in its own XY plane; the 90 deg cant and the stage
            # rotation live in the TF chain, not here.
            local = np.empty((r.size, 4))
            local[:, 0] = r * np.cos(a)
            local[:, 1] = r * np.sin(a)
            local[:, 2] = 0.0
            local[:, 3] = 1.0

            base_h = (transform_to_mat(tf_bl) @ local.T).T     # in base_frame
            world = (T_fix_base @ base_h.T).T[:, :3]

            if self.mask_frame == self.base_frame:
                bod = base_h[:, :3]
            else:
                tf_ml, _ = self.lookup_stamped(
                    self.mask_frame, scan.header.frame_id, stamp)
                bod = None if tf_ml is None else \
                    (transform_to_mat(tf_ml) @ local.T).T[:, :3]

            if bod is None:
                world_keep.append(world)
                continue

            if self.publish_body:
                body_all.append(bod)

            if enabled:
                inside = ((bod[:, 0] >= x0) & (bod[:, 0] <= x1) &
                          (bod[:, 1] >= y0) & (bod[:, 1] <= y1) &
                          (bod[:, 2] >= z0) & (bod[:, 2] <= z1))
                self.n_masked += int(inside.sum())
                world = world[~inside]

            if world.size:
                world_keep.append(world)

        if not world_keep and not body_all:
            self.n_dropped += 1
            return

        self.n_scans += 1
        if degraded_any:
            self.n_degraded += 1
        if world_keep:
            pts = np.vstack(world_keep)
            self.n_kept += pts.shape[0]
            self.insert(self.grid, pts, self.voxel, self.max_voxels, warn=True)
        if body_all:
            self.insert(self.body, np.vstack(body_all),
                        self.body_voxel, self.body_max)

    def insert(self, grid, pts, voxel, cap, warn=False):
        keys = np.floor(pts / voxel).astype(np.int32)
        for k, p in zip(map(tuple, keys), pts):
            if k in grid:
                continue
            if len(grid) >= cap:
                if warn and not self.capped:
                    self.get_logger().warn(
                        f"voxel cap {cap} reached — no longer growing "
                        "(raise max_voxels or voxel_size)")
                    self.capped = True
                return
            grid[k] = p

    def publish_clouds(self):
        stamp = self.get_clock().now().to_msg()
        if self.grid:
            pts = np.array(list(self.grid.values()), dtype=np.float32)
            self.pub.publish(make_cloud_msg(pts, self.fixed_frame, stamp))
        if self.publish_body and self.body:
            pts = np.array(list(self.body.values()), dtype=np.float32)
            self.body_pub.publish(make_cloud_msg(pts, self.mask_frame, stamp))

    def log_health(self):
        msg = (f"scans={self.n_scans} points={len(self.grid)} "
               f"dropped={self.n_dropped} masked={self.n_masked}")
        if self.n_degraded:
            msg += f" degraded_tf={self.n_degraded}"
        if self._pose_xyz is not None:
            x, y, z = self._pose_xyz
            msg += (f" | pose x={x:+.2f} y={y:+.2f} z={z:+.2f} "
                    f"yaw={self._pose_yaw:+.1f}")
        if self.n_dropped > self.n_scans and self.n_scans < 5:
            msg += f"  <-- TF lookups failing: {self._last_tf_error}"
        if self.n_kept and self.n_masked > 3 * self.n_kept:
            msg += "  <-- mask box may be far too large"
        self.get_logger().info(msg)

    # ---------- services ----------

    def save_pcd(self, path=None):
        if not self.grid:
            return None
        path = path or f"cloud_3d_{int(time.time())}.pcd"
        pts = np.array(list(self.grid.values()), dtype=np.float32)
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n"
                    "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    "COUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\nHEIGHT 1\n"
                    "VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\nDATA ascii\n")
            for p in pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        return path

    def on_save(self, req, res):
        path = self.save_pcd()
        res.success = path is not None
        res.message = f"wrote {path}" if path else "cloud is empty"
        self.get_logger().info(res.message)
        return res

    def on_clear(self, req, res):
        self.grid.clear()
        self.body.clear()
        self.capped = False
        res.success = True
        res.message = "clouds cleared"
        return res


def main():
    rclpy.init()
    node = ScanAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        path = node.save_pcd()
        if path:
            node.get_logger().info(f"wrote {path} on shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
