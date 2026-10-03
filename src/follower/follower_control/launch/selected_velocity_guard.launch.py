"""Connect the command selector output to the final Follower safety guard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the guard in selector-integrated software-control mode."""
    config = PathJoinSubstitution(
        [FindPackageShare("follower_control"), "config", "velocity_guard.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "guard_enabled_on_startup",
                default_value="false",
                choices=["true", "false"],
                description="Enable final guarded velocity output on startup",
            ),
            DeclareLaunchArgument(
                "allow_reverse",
                default_value="false",
                choices=["true", "false"],
                description="Allow negative linear velocity for cooperative motion",
            ),
            Node(
                package="follower_control",
                executable="velocity_guard_node",
                namespace="/follower",
                name="velocity_guard",
                parameters=[
                    config,
                    {
                        "command_topic": "/follower/selected_cmd_vel",
                        "guard_enabled_on_startup": ParameterValue(
                            LaunchConfiguration("guard_enabled_on_startup"),
                            value_type=bool,
                        ),
                        "allow_reverse": ParameterValue(
                            LaunchConfiguration("allow_reverse"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            )
        ]
    )
