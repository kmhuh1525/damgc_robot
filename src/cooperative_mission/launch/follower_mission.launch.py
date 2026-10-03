"""Follower Orin: perception, guarded command path, gripper and mission executor.

The command selector starts in STOP and its COOPERATION input is remapped to
``/follower/mission/cmd_vel`` (owned by the mission executor), so the
Follower only moves on acknowledged Leader mission commands.

Do NOT run follower_cooperation_drive / follower_apriltag_drive /
gripper_sequence at the same time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, launch_arguments=None, condition=None):
    launch_path = os.path.join(get_package_share_directory(package_name), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("cooperative_mission")
    config = LaunchConfiguration

    arguments = [
        DeclareLaunchArgument(
            "mission_config",
            default_value=PathJoinSubstitution([share, "config", "follower_mission.yaml"]),
            description="Follower mission executor parameter file",
        ),
        DeclareLaunchArgument(
            "approach_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("follower_supply_perception"), "config", "approach.yaml"]
            ),
            description="Follower AprilTag approach parameters (target tag of the far face)",
        ),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("use_stm32_bridge", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
        DeclareLaunchArgument("i2c_address", default_value="66"),
        DeclareLaunchArgument("i2c_write_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gripper_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("rx64_speed", default_value="50"),
        DeclareLaunchArgument("gripper_open_raw", default_value="950"),
        DeclareLaunchArgument("gripper_close_raw", default_value="350"),
        DeclareLaunchArgument("lift_raw", default_value="300"),
        DeclareLaunchArgument("lower_raw", default_value="600"),
    ]

    selector = Node(
        package="follower_command_selector",
        executable="command_selector_node",
        namespace="/follower",
        name="command_selector",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("follower_command_selector"), "config", "command_selector.yaml"]
            ),
            {"source_mode": "STOP"},
        ],
        # COOPERATION source = mission executor (reposition/search/transport).
        remappings=[("cmd_vel", "/follower/mission/cmd_vel")],
    )

    stm32_bridge = GroupAction(
        condition=IfCondition(config("use_stm32_bridge")),
        scoped=True,
        actions=[
            SetRemap(src="cmd_vel", dst="/follower/safe_cmd_vel"),
            _include(
                "stm32_bridge",
                "stm32_bridge.launch.py",
                {
                    "transport": "i2c",
                    "i2c_device": config("i2c_device"),
                    "i2c_address": config("i2c_address"),
                    "i2c_write_enabled": config("i2c_write_enabled"),
                    "namespace": "follower",
                },
            ),
        ],
    )

    mission = Node(
        package="cooperative_mission",
        executable="follower_mission_node",
        namespace="follower",
        name="mission_executor",
        output="screen",
        parameters=[
            config("mission_config"),
            {
                "require_gripper": ParameterValue(config("gripper_enabled"), value_type=bool),
                "gripper_open_raw": ParameterValue(config("gripper_open_raw"), value_type=int),
                "gripper_close_raw": ParameterValue(config("gripper_close_raw"), value_type=int),
                "lift_raw": ParameterValue(config("lift_raw"), value_type=int),
                "lower_raw": ParameterValue(config("lower_raw"), value_type=int),
            },
        ],
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(msg=(
                "Follower mission stack: selector STOP, guard disabled; waiting for "
                "Leader mission commands on /mission/follower_command."
            )),
            _include(
                "follower_supply_perception",
                "follower_apriltag.launch.py",
                {
                    "approach_config": config("approach_config"),
                    "video_device": config("video_device"),
                },
            ),
            _include(
                "follower_approach_control",
                "approach_controller.launch.py",
                {"enabled_on_startup": "false"},
            ),
            selector,
            _include(
                "follower_control",
                "selected_velocity_guard.launch.py",
                {
                    "guard_enabled_on_startup": "false",
                    "allow_reverse": "true",
                },
            ),
            stm32_bridge,
            _include(
                "rescue_robot_tools",
                "dynamixel_orin.launch.py",
                {
                    "robot": "follower",
                    "port": config("gripper_port"),
                    "rx64_speed": config("rx64_speed"),
                    "startup_pose_enabled": "false",
                    "startup_torque": "false",
                },
                condition=IfCondition(config("gripper_enabled")),
            ),
            mission,
        ]
    )
