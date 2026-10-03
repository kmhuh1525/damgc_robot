"""Launch only the survivor camera-to-map transform consumer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose the transform topics, target frame, and lookup timeout."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/leader/survivor/camera_positions",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/leader/survivor/map_positions",
            ),
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("tf_timeout_sec", default_value="0.6"),
            Node(
                package="rescue_robot_survivor",
                executable="survivor_map_transform_node",
                namespace="leader",
                name="survivor_map_transform",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "tf_timeout_sec": ParameterValue(
                            LaunchConfiguration("tf_timeout_sec"),
                            value_type=float,
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
