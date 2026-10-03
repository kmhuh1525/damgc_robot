"""Launch the guarded Follower command path for cooperative transport."""

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


def _include(package_name, launch_file, launch_arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Start the Follower selector, final guard, and optional motor bridge."""
    use_stm32_bridge = LaunchConfiguration("use_stm32_bridge")
    guard_enabled_on_startup = LaunchConfiguration("guard_enabled_on_startup")
    i2c_device = LaunchConfiguration("i2c_device")
    i2c_address = LaunchConfiguration("i2c_address")
    i2c_write_enabled = LaunchConfiguration("i2c_write_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_stm32_bridge",
                default_value="true",
                choices=["true", "false"],
                description="Launch the Follower STM32 command bridge",
            ),
            DeclareLaunchArgument(
                "guard_enabled_on_startup",
                default_value="false",
                choices=["true", "false"],
                description="Open the final Follower velocity gate at startup",
            ),
            DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
            DeclareLaunchArgument("i2c_address", default_value="66"),
            DeclareLaunchArgument(
                "i2c_write_enabled",
                default_value="true",
                choices=["true", "false"],
            ),
            LogInfo(
                msg=(
                    "Follower cooperation startup safety: selector COOPERATION, "
                    "velocity guard disabled by default."
                )
            ),
            _include(
                "follower_command_selector",
                "command_selector.launch.py",
                {"source_mode": "COOPERATION"},
            ),
            _include(
                "follower_control",
                "selected_velocity_guard.launch.py",
                {
                    "guard_enabled_on_startup": guard_enabled_on_startup,
                    "allow_reverse": "true",
                },
            ),
            GroupAction(
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
            ),
        ]
    )
