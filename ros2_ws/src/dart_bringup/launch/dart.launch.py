from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("dart_bringup"))
    config = LaunchConfiguration("config")
    camera_info_url = f"file://{share / 'config' / 'camera_info_standard3.yaml'}"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "standard3.yaml")
            ),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("detect", default_value="true"),
            DeclareLaunchArgument("mcu", default_value="true"),
            DeclareLaunchArgument("estimate", default_value="true"),
            Node(
                package="camera_node",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[config, {"camera_info_url": camera_info_url}],
                condition=IfCondition(LaunchConfiguration("camera")),
            ),
            Node(
                package="detect_node",
                executable="detect_node",
                name="detect_node",
                output="screen",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("detect")),
            ),
            Node(
                package="mcu_node",
                executable="mcu_node",
                name="mcu_node",
                output="screen",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("mcu")),
            ),
            Node(
                package="estimate_node",
                executable="estimate_node",
                name="estimate_node",
                output="screen",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("estimate")),
            ),
        ]
    )
