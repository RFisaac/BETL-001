"""Start the whole BETL voice system:  ros2 launch betl_voice betl.launch.py

Optional arguments:
  config:=/path/to/betl.yaml   use a different settings file
  leds:=false                  run without the LED ring
  llm:=true                    also start the local language model server for chat
                               (needs scripts/setup_llm.sh first)
  llm_threads:=4               CPU threads for the language model
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory("betl_voice"), "config", "betl.yaml")
    cfg = LaunchConfiguration("config")
    llm_dir = os.path.expanduser("~/betl_models/llm")

    def node(executable, name, **kw):
        return Node(package="betl_voice", executable=executable, name=name, namespace="betl",
                    parameters=[cfg], output="screen", emulate_tty=True, **kw)

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_cfg),
        DeclareLaunchArgument("leds", default_value="true"),
        DeclareLaunchArgument("llm", default_value="false"),
        DeclareLaunchArgument("llm_threads", default_value="4"),
        node("respeaker_node", "respeaker"),
        node("listener_node", "listener"),
        node("brain_node", "brain"),
        node("tts_node", "tts"),
        node("led_node", "led", condition=IfCondition(LaunchConfiguration("leds"))),
        ExecuteProcess(
            cmd=[os.path.join(llm_dir, "llama-server"), "-m", os.path.join(llm_dir, "model.gguf"),
                 "-t", LaunchConfiguration("llm_threads"), "-c", "2048", "--host", "127.0.0.1",
                 "--port", "8080"],
            name="llm", output="log", condition=IfCondition(LaunchConfiguration("llm"))),
    ])
