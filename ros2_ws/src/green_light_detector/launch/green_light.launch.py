from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("green_light_detector"))
        / "config"
        / "standard3_green_light.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="green_light_detector",
                executable="green_light_node",
                name="green_light_detector",
                output="screen",
                parameters=[LaunchConfiguration("config")],
                ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
