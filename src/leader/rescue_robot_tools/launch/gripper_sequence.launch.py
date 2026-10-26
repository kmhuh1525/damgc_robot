from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("task_mode", default_value="false"),
        DeclareLaunchArgument("open_wait", default_value="17.0"),
        DeclareLaunchArgument("robot", default_value="leader"),
        DeclareLaunchArgument("node_namespace", default_value=""),
        DeclareLaunchArgument("detection_topic", default_value="/leader/supply/detected"),
        DeclareLaunchArgument(
            "alignment_topic", default_value="/leader/base_alignment/state"
        ),
        DeclareLaunchArgument("raw_command_topic", default_value="/leader/dynamixel/command"),
        DeclareLaunchArgument("gripper_topic", default_value="/leader/gripper/command"),
        DeclareLaunchArgument("open_raw", default_value="950"),
        DeclareLaunchArgument("close_raw", default_value="350"),
        DeclareLaunchArgument("close_wait", default_value="3.0"),
        DeclareLaunchArgument("lift_enabled", default_value="false"),
        DeclareLaunchArgument("lift_raw", default_value="-1.0"),
        DeclareLaunchArgument("lost_rx64_raw", default_value="600"),
        DeclareLaunchArgument("lost_rx28_raw", default_value="500"),
        DeclareLaunchArgument("tag_lost_idle_enabled", default_value="true"),
        Node(
            package="rescue_robot_tools",
            executable="gripper_sequence_node.py",
            name="gripper_sequence",
            namespace=LaunchConfiguration("node_namespace"),
            output="screen",
            parameters=[{
                "enabled": ParameterValue(
                    LaunchConfiguration("enabled"), value_type=bool
                ),
                "robot": LaunchConfiguration("robot"),
                "task_mode": ParameterValue(LaunchConfiguration("task_mode"), value_type=bool),
                "open_wait": ParameterValue(LaunchConfiguration("open_wait"), value_type=float),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "alignment_topic": LaunchConfiguration("alignment_topic"),
                "raw_command_topic": LaunchConfiguration("raw_command_topic"),
                "gripper_topic": LaunchConfiguration("gripper_topic"),
                "open_raw": ParameterValue(
                    LaunchConfiguration("open_raw"), value_type=int
                ),
                "close_raw": ParameterValue(
                    LaunchConfiguration("close_raw"), value_type=int
                ),
                "close_wait": ParameterValue(
                    LaunchConfiguration("close_wait"), value_type=float
                ),
                "lift_enabled": ParameterValue(
                    LaunchConfiguration("lift_enabled"), value_type=bool
                ),
                "lift_raw": ParameterValue(
                    LaunchConfiguration("lift_raw"), value_type=float
                ),
                "lost_rx64_raw": ParameterValue(
                    LaunchConfiguration("lost_rx64_raw"), value_type=int
                ),
                "lost_rx28_raw": ParameterValue(
                    LaunchConfiguration("lost_rx28_raw"), value_type=int
                ),
                "tag_lost_idle_enabled": ParameterValue(
                    LaunchConfiguration("tag_lost_idle_enabled"), value_type=bool
                ),
            }],
        ),
    ])
