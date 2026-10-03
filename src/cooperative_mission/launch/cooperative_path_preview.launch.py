"""Diagnostic-only follower path preview derived from the Leader Nav2 plan."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cooperative_mission"), "config", "cooperative_path_preview.yaml"]
                ),
                description="Measured rigid-grasp geometry and preview topics",
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_path_preview_node",
                name="cooperative_path_preview",
                output="screen",
                parameters=[config],
            ),
        ]
    )
