#!/usr/bin/env python3
"""
beetle_bringup.launch.py — the whole X5 stack in one command.

    ros2 launch /userdata/beetle_bringup.launch.py

    # driving demo only, no scanning:
    ros2 launch /userdata/beetle_bringup.launch.py lidar:=false cloud:=false

    # with the calibration values baked in:
    ros2 launch /userdata/beetle_bringup.launch.py gyro_scale:=1.023 stage_zero:=-4.5

Ctrl-C once shuts everything down cleanly.

WHAT THIS CANNOT DO, AND WHY IT MATTERS
  The clean-bringup procedure is a chain of GATES — each step verifies its
  dependency is really alive before the next starts. A launch file cannot
  check a gate; it can only wait. So the ordering below is enforced with
  delays sized generously against measured startup times.

  That is fine for a demo on known-good hardware. It is NOT a substitute for
  the gated procedure when something is broken: launch will happily start the
  accumulator against a dead TF chain and you will get a silent, empty cloud
  instead of an error. When debugging, bring it up by hand.

TIMELINE
  t=0    camera, drives, static TFs, lidar, stage, rosbridge, console server
  t=14   wheel odometry      (camera enumerated ~10 s; /wheel_states flowing)
  t=30   accumulator         (TF chain up, gyro bias converging)
  t=34   stage spin-up       (only if spin:=true)

ORDERING CONSTRAINTS THAT ARE REAL
  - wheel_odom subscribes /wheel_states, which ONLY the drive node publishes.
    Start it first or wheel_odom sits there doing nothing, quietly.
  - wheel_odom needs /camera/camera/imu for yaw. No IMU -> silent fallback to
    scrub-corrupted wheels-only heading.
  - the accumulator needs the WHOLE chain odom -> camera_link -> stage_base
    -> stage_mount -> laser. Started early it drops every scan.

AFTER LAUNCH, CHECK THESE TWO THINGS
    ros2 run tf2_ros tf2_echo odom laser     # must stream AND rotate
    # wheel_odom health line must read yaw_src=GYRO, not wheels
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import (PythonLaunchDescriptionSource,
                                               AnyLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SCRIPTS = "/userdata"
CLOUD_DIR = "/userdata/beetle/clouds"


def generate_launch_description():
    lc = LaunchConfiguration

    args = [
        # --- subsystems ---
        DeclareLaunchArgument("lidar", default_value="true",
                              description="rplidar + rotating stage"),
        DeclareLaunchArgument("cloud", default_value="true",
                              description="scan accumulator (needs lidar)"),
        DeclareLaunchArgument("console", default_value="true",
                              description="rosbridge + HTTP server for beetle_console.html"),
        DeclareLaunchArgument("foxglove", default_value="false",
                              description="foxglove_bridge — costs ~1 CPU core, off by default"),
        DeclareLaunchArgument("yolo", default_value="false",
                              description="BPU detection + overlay"),
        DeclareLaunchArgument("spin", default_value="false",
                              description="start the stage turning automatically"),

        # --- calibration, all live-tunable from the console afterwards ---
        DeclareLaunchArgument("gyro_scale", default_value="1.0",
                              description="heading gain; measure with the console's yaw calibration"),
        DeclareLaunchArgument("stage_zero", default_value="0.0",
                              description="stage zero offset, degrees"),
        DeclareLaunchArgument("max_wheel_tps", default_value="0.6",
                              description="speed cap, turns/s. 0.6 ~ 0.29 m/s at r=0.0765"),

        # --- hardware ---
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar",
                              description="udev symlink — NEVER /dev/ttyUSBn, order is not stable"),
        DeclareLaunchArgument("stage_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("console_dir", default_value="/root/beetle_hud"),
    ]

    # ---------------------------------------------------------------- t = 0

    # RGB + IMU only. Depth plus align_depth put RealSense at 111% CPU and
    # took load average to 16.4, which broke everything downstream. The only
    # thing depth was feeding was RTAB-Map's visual odometry, and wheel+gyro
    # odometry replaced that.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "enable_depth": "false",
            "enable_infra": "false",
            "align_depth.enable": "false",
            # NOTE: rgb_camera.color_profile, NOT rgb_camera.profile. The old
            # name is silently ignored and you get 1280x720 without warning.
            "rgb_camera.color_profile": "640x480x15",
        }.items(),
    )

    drives = ExecuteProcess(
        cmd=["python3", f"{SCRIPTS}/odesc_tuner_node.py", "--ros-args",
             "-p", ["max_wheel_tps:=", lc("max_wheel_tps")]],
        cwd=SCRIPTS, output="screen", name="odesc_tuner",
    )

    # Mounting offset only. The stage ZERO used to live in this publisher's
    # --yaw; it now lives in scan_stage_node's angle_offset_deg so it can be
    # tuned live. Leave this yaw at 0.
    tf_stage_base = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="tf_camera_to_stage_base", output="log",
        arguments=["--x", "-0.117", "--y", "0.057", "--z", "0.038",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "camera_link", "--child-frame-id", "stage_base"],
    )

    # The 90 degree lidar cant — this is what makes the scan plane vertical
    # and the whole spherical-scanning idea work.
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="tf_stage_mount_to_laser", output="log",
        arguments=["--x", "0", "--y", "0", "--z", "0.0505",
                   "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
                   "--frame-id", "stage_mount", "--child-frame-id", "laser"],
    )

    lidar = Node(
        package="rplidar_ros", executable="rplidar_node", name="rplidar_node",
        output="screen", condition=IfCondition(lc("lidar")),
        parameters=[{"serial_port": lc("lidar_port"),
                     "serial_baudrate": 115200,
                     "frame_id": "laser"}],
    )

    stage = Node(
        package="scan_stage", executable="scan_stage_node", name="scan_stage_node",
        output="screen", condition=IfCondition(lc("lidar")),
        parameters=[{"port": lc("stage_port"),
                     "base_frame": "stage_base",
                     "lidar_frame": "stage_mount",
                     "angle_offset_deg": lc("stage_zero"),
                     # Pico ticks_us() wraps every 17.9 min and the offset
                     # estimator is not wrap-aware; a short window shrinks the
                     # damage from 30 s of bad transforms to 5 s.
                     "offset_reset_sec": 5.0}],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rosbridge_server"), "launch",
             "rosbridge_websocket_launch.xml"])),
        condition=IfCondition(lc("console")),
        # Globs must be permissive: every console knob is a set_parameters
        # service call, and rosbridge blocks services by default.
        # rosbridge declares these as STRING and parses the brackets itself.
        # Bare [*] fails yaml.safe_load (* is an alias marker); ['*'] yields a
        # STRING_ARRAY the node rejects. The quotes must be part of the VALUE
        # so yaml resolves it to the plain string [*].
        launch_arguments={"topics_glob": "'[*]'",
                          "services_glob": "'[*]'",
                          "params_glob": "'[*]'"}.items(),
    )

    console_http = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "8080"],
        cwd=lc("console_dir"), output="log", name="console_http",
        condition=IfCondition(lc("console")),
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("foxglove_bridge"), "launch",
             "foxglove_bridge_launch.xml"])),
        condition=IfCondition(lc("foxglove")),
    )

    # --------------------------------------------------------------- t = 14
    # Camera needs ~10 s to enumerate, and the drive node must already be
    # publishing /wheel_states.
    odom = TimerAction(period=14.0, actions=[
        LogInfo(msg="[beetle] starting wheel odometry"),
        ExecuteProcess(
            cmd=["python3", f"{SCRIPTS}/wheel_odom_node.py", "--ros-args",
                 "-p", ["gyro_scale:=", lc("gyro_scale")]],
            cwd=SCRIPTS, output="screen", name="wheel_odom"),
    ])

    # --------------------------------------------------------------- t = 30
    # mask_frame == base_frame halves the TF lookups per scan. The box bounds
    # are the stage_base box shifted by the camera_link->stage_base
    # translation, which is exact ONLY while that publisher's yaw is 0.
    cloud = TimerAction(period=30.0, actions=[
        LogInfo(msg="[beetle] starting accumulator — check tf2_echo odom laser"),
        ExecuteProcess(
            cmd=["python3", f"{SCRIPTS}/scan_accumulator_node.py", "--ros-args",
                 "-p", "fixed_frame:=odom",
                 "-p", "base_frame:=camera_link",
                 "-p", "mask_frame:=camera_link",
                 "-p", "mask_min_x:=-0.717", "-p", "mask_max_x:=0.133",
                 "-p", "mask_min_y:=-0.243", "-p", "mask_max_y:=0.357",
                 "-p", "mask_min_z:=-0.962", "-p", "mask_max_z:=0.088",
                 "-p", "segments:=8",
                 # Do NOT raise tf_timeout. lookup_transform blocks the
                 # calling thread; at 0.15 s throughput fell from 5.9 to 0.9
                 # scans/s against a 6.42 Hz lidar.
                 "-p", "tf_timeout:=0.05",
                 "-p", "voxel_size:=0.05",
                 "-p", "publish_hz:=1.0",
                 "-p", "max_voxels:=200000"],
            cwd=CLOUD_DIR, output="screen", name="scan_accumulator",
            condition=IfCondition(lc("cloud"))),
    ])

    yolo = TimerAction(period=18.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "run", "dnn_node_example", "example", "--ros-args",
                 "-p", "config_file:=config/yolov8workconfig.json",
                 "-p", "feed_type:=1",
                 # shared-memory subscriber fights the RealSense; 0 switches
                 # it to a plain ROS topic subscriber
                 "-p", "is_shared_mem_sub:=0",
                 "-p", "ros_img_topic_name:=/camera/camera/color/image_raw",
                 "-p", "image_width:=640", "-p", "image_height:=480",
                 "-p", "dump_render_img:=0",
                 "-p", "msg_pub_topic_name:=/beetle_detections",
                 "--log-level", "warn"],
            cwd=SCRIPTS, output="screen", name="yolo",
            condition=IfCondition(lc("yolo"))),
        ExecuteProcess(
            cmd=["python3", f"{SCRIPTS}/detection_overlay.py", "--ros-args",
                 # 10 Hz of raw rgb8 is ~9 MB/s through the bridge. 2 Hz is
                 # plenty to show detections working.
                 "-p", "publish_hz:=2.0"],
            cwd=SCRIPTS, output="log", name="detection_overlay",
            condition=IfCondition(lc("yolo"))),
    ])

    # --------------------------------------------------------------- t = 34
    spin = TimerAction(period=34.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "service", "call", "/stage/spin", "std_srvs/srv/Trigger"],
            output="screen", name="stage_spin",
            condition=IfCondition(lc("spin"))),
    ])

    ready = TimerAction(period=32.0, actions=[LogInfo(msg=(
        "\n"
        "  ==========================================================\n"
        "   BEETLE UP.  Deck:  python3 deck_teleop.py --host <X5_IP>\n"
        "   Console:    http://<X5_IP>:8080/beetle_console.html\n"
        "\n"
        "   VERIFY BEFORE DRIVING:\n"
        "     ros2 run tf2_ros tf2_echo odom laser   (streams AND rotates)\n"
        "     wheel_odom health line says yaw_src=GYRO\n"
        "  =========================================================="))])

    return LaunchDescription(args + [
        LogInfo(msg="[beetle] bringup — camera ~10 s, odometry at 14 s, cloud at 30 s"),
        camera, drives, tf_stage_base, tf_laser,
        lidar, stage, rosbridge, console_http, foxglove,
        odom, yolo, cloud, ready, spin,
    ])
