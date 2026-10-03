"""Preview follower axle path derived from the Leader Nav2 /plan topic."""

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
                    [
                        FindPackageShare("cooperative_mission"),
                        "config",
                        "cooperative_path_preview.yaml",
                    ]
                ),
                description="Passive hinge geometry and path topics",
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_leader_path_adapter_node",
                name="cooperative_leader_path_adapter",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_path_preview_node",
                name="cooperative_path_preview",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_path_tracking_preview_node",
                name="cooperative_path_tracking_preview",
                output="screen",
                parameters=[config],
            ),
        ]
    )
