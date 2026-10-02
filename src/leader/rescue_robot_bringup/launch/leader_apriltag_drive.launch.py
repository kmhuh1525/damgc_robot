import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, launch_arguments, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    """Launch the complete, guarded Leader AprilTag drive pipeline."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_enabled", default_value="true"),
            DeclareLaunchArgument("gripper_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
            DeclareLaunchArgument("rx64_speed", default_value="50"),
            DeclareLaunchArgument("gripper_open_raw", default_value="1000"),
            DeclareLaunchArgument("gripper_close_raw", default_value="450"),
            DeclareLaunchArgument("lift_enabled", default_value="true"),
            DeclareLaunchArgument("lift_raw", default_value="300"),
            DeclareLaunchArgument("gripper_lost_rx64_raw", default_value="600"),
            DeclareLaunchArgument("gripper_lost_rx28_raw", default_value="500"),
            DeclareLaunchArgument(
                "final_target_distance",
                default_value="0.23",
                description="Final tag-normal distance from tag plane to base_link",
            ),
            DeclareLaunchArgument("post_align_odom_enabled", default_value="true"),
            DeclareLaunchArgument("post_align_grasp_target_distance", default_value="0.16"),
            LogInfo(
                msg=(
                    "Leader AprilTag drive startup safety: approach controller "
                    "ENABLED, velocity guard DISABLED, motor command held at zero. "
                    "Call /leader/velocity_guard/enable with data=true to drive."
                )
            ),
            _include(
                "rescue_robot_bringup",
                "camera_apriltag.launch.py",
                {
                    "enable_depth": "true",
                    "enable_infra": "false",
                    "enable_imu": "false",
                    "enable_approach": "true",
                    "final_target_distance": LaunchConfiguration(
                        "final_target_distance"
                    ),
                    "post_align_odom_enabled": LaunchConfiguration(
                        "post_align_odom_enabled"
                    ),
                    "post_align_grasp_target_distance": LaunchConfiguration(
                        "post_align_grasp_target_distance"
                    ),
                },
            ),
            _include(
                "leader_approach_control",
                "approach_controller.launch.py",
                {"controller_enabled_on_startup": "true"},
            ),
            _include(
                "leader_approach_control",
                "velocity_guard.launch.py",
                {"guard_enabled_on_startup": "false"},
            ),
            _include(
                "stm32_bridge",
                "stm32_bridge.launch.py",
                {
                    "transport": "i2c",
                    "i2c_device": "/dev/i2c-7",
                    "i2c_address": "66",
                    "i2c_write_enabled": "true",
                    "namespace": "leader",
                },
            ),
            _include(
                "rescue_robot_tools",
                "dynamixel_orin.launch.py",
                {
                    "robot": "leader",
                    "port": LaunchConfiguration("gripper_port"),
                    "baudrate": LaunchConfiguration("gripper_baudrate"),
                    "rx64_speed": LaunchConfiguration("rx64_speed"),
                    # Keep the shared child defaults unchanged, but prevent the
                    # integrated Leader launch from writing an arbitrary startup
                    # pose or enabling torque before a targeted command exists.
                    "startup_pose_enabled": "false",
                    "startup_torque": "false",
                },
                condition=IfCondition(LaunchConfiguration("gripper_enabled")),
            ),
            _include(
                "rescue_robot_tools",
                "gripper_sequence.launch.py",
                {
                    "enabled": "true",
                    "detection_topic": "/leader/supply/detected",
                    "alignment_topic": "/leader/base_alignment/state",
                    "open_raw": LaunchConfiguration("gripper_open_raw"),
                    "close_raw": LaunchConfiguration("gripper_close_raw"),
                    "lift_enabled": LaunchConfiguration("lift_enabled"),
                    "lift_raw": LaunchConfiguration("lift_raw"),
                    "lost_rx64_raw": LaunchConfiguration("gripper_lost_rx64_raw"),
                    "lost_rx28_raw": LaunchConfiguration("gripper_lost_rx28_raw"),
                    # A transient detector loss must not reposition either
                    # actuator; retain the last commanded gripper position.
                    "tag_lost_idle_enabled": "false",
                },
                condition=IfCondition(LaunchConfiguration("gripper_enabled")),
            ),
        ]
    )
