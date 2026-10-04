"""Launch the complete, guarded Follower AprilTag drive pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _include(package_name, launch_file, launch_arguments=None, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    """Launch perception through the guarded Follower STM32 command bridge."""
    use_stm32_bridge = LaunchConfiguration("use_stm32_bridge")
    i2c_device = LaunchConfiguration("i2c_device")
    i2c_address = LaunchConfiguration("i2c_address")
    i2c_write_enabled = LaunchConfiguration("i2c_write_enabled")

    arguments = [
        DeclareLaunchArgument(
            "gripper_enabled",
            default_value="true",
            choices=["true", "false"],
            description="Launch the Follower Dynamixel gripper subsystem",
        ),
        DeclareLaunchArgument(
            "gripper_port",
            default_value="/dev/ttyUSB0",
            description="Follower U2D2 serial device",
        ),
        DeclareLaunchArgument(
            "gripper_open_raw",
            default_value="950",
            description="Follower RX-28 OPEN raw goal",
        ),
        DeclareLaunchArgument(
            "gripper_close_raw",
            default_value="350",
            description="Follower RX-28 CLOSE raw goal",
        ),
        DeclareLaunchArgument(
            "rx64_speed",
            default_value="50",
            description="Follower RX-64 Moving Speed raw value",
        ),
        DeclareLaunchArgument(
            "lift_enabled",
            default_value="true",
            choices=["true", "false"],
            description="Enable automatic Follower RX-64 lift after close",
        ),
        DeclareLaunchArgument(
            "lift_raw",
            default_value="20",
            description="Follower RX-64 lift raw goal (valid range 20..270; 20=raised, 270=lowered)",
        ),
        DeclareLaunchArgument(
            "use_stm32_bridge",
            default_value="true",
            choices=["true", "false"],
            description="Launch the Follower STM32 command bridge",
        ),
        DeclareLaunchArgument(
            "i2c_device",
            default_value="/dev/i2c-7",
            description="Follower STM32 I2C device",
        ),
        DeclareLaunchArgument(
            "i2c_address",
            default_value="66",
            description="Follower STM32 I2C slave address (66 is 0x42)",
        ),
        DeclareLaunchArgument(
            "i2c_write_enabled",
            default_value="true",
            choices=["true", "false"],
            description="Enable velocity-frame writes over Follower I2C",
        ),
    ]

    stm32_bridge = GroupAction(
        condition=IfCondition(use_stm32_bridge),
        scoped=True,
        actions=[
            SetRemap(src="cmd_vel", dst="/follower/safe_cmd_vel"),
            _include(
                "stm32_bridge",
                "stm32_bridge.launch.py",
                {
                    "transport": "i2c",
                    "i2c_device": i2c_device,
                    "i2c_address": i2c_address,
                    "i2c_write_enabled": i2c_write_enabled,
                    "namespace": "follower",
                },
            ),
        ],
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=(
                    "Follower AprilTag drive startup safety: approach controller "
                    "ENABLED, command selector APPROACH, velocity guard DISABLED. "
                    "Call /follower/velocity_guard/enable with data=true to drive."
                )
            ),
            _include(
                "follower_supply_perception",
                "follower_apriltag.launch.py",
            ),
            _include(
                "follower_approach_control",
                "approach_controller.launch.py",
                {"enabled_on_startup": "true"},
            ),
            _include(
                "follower_command_selector",
                "command_selector.launch.py",
                {"source_mode": "APPROACH"},
            ),
            _include(
                "follower_control",
                "selected_velocity_guard.launch.py",
                {"guard_enabled_on_startup": "false"},
            ),
            stm32_bridge,
            _include(
                "rescue_robot_tools",
                "dynamixel_orin.launch.py",
                {
                    "robot": "follower",
                    "port": LaunchConfiguration("gripper_port"),
                    "rx64_speed": LaunchConfiguration("rx64_speed"),
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
                    "robot": "follower",
                    "node_namespace": "follower",
                    "detection_topic": "/follower/supply/detected",
                    "alignment_topic": "/follower/base_alignment/state",
                    "raw_command_topic": "/follower/dynamixel/command",
                    "gripper_topic": "/follower/gripper/command",
                    "open_raw": LaunchConfiguration("gripper_open_raw"),
                    "close_raw": LaunchConfiguration("gripper_close_raw"),
                    "lift_enabled": LaunchConfiguration("lift_enabled"),
                    "lift_raw": LaunchConfiguration("lift_raw"),
                    "tag_lost_idle_enabled": "false",
                },
                condition=IfCondition(LaunchConfiguration("gripper_enabled")),
            ),
        ]
    )
