"""Leader Orin: complete cooperative grasp-lift-transport mission stack.

Starts the existing Leader AprilTag perception and approach controller, the
Leader velocity guard (input owned by the mission coordinator), the Leader
command selector, the STM32 bridge, the Dynamixel gripper node and the mission
coordinator itself.
Nothing moves until ``/mission/start`` is called.

Do NOT run leader_cooperation / leader_apriltag_drive / gripper_sequence at
the same time: they publish the same command topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
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
    direction = config("transport_direction")

    arguments = [
        DeclareLaunchArgument(
            "mission_config",
            default_value=PathJoinSubstitution([share, "config", "leader_mission.yaml"]),
            description="Leader mission coordinator parameter file",
        ),
        DeclareLaunchArgument(
            "guard_config",
            default_value=PathJoinSubstitution(
                [share, "config", "leader_velocity_guard_mission.yaml"]
            ),
            description="Leader velocity guard parameters for the mission",
        ),
        DeclareLaunchArgument(
            "transport_direction", default_value="forward", choices=["forward", "backward"],
            description="Transport direction in the Leader base_link frame",
        ),
        DeclareLaunchArgument("transport_speed", default_value="0.05"),
        DeclareLaunchArgument("transport_duration", default_value="1.0"),
        DeclareLaunchArgument(
            "start_command_selector", default_value="true", choices=["true", "false"],
            description=(
                "Start the Leader selector in MISSION mode; set false only when "
                "a shared selector is already running in MISSION mode"
            ),
        ),
        DeclareLaunchArgument(
            "final_target_distance", default_value="0.23",
            description="Final tag-normal distance from tag plane to Leader base_link",
        ),
        DeclareLaunchArgument("use_stm32_bridge", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-7"),
        DeclareLaunchArgument("i2c_address", default_value="66"),
        DeclareLaunchArgument("i2c_write_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gripper_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
        DeclareLaunchArgument("rx64_speed", default_value="50"),
        DeclareLaunchArgument("gripper_open_raw", default_value="1000"),
        DeclareLaunchArgument("gripper_close_raw", default_value="450"),
        DeclareLaunchArgument("lift_raw", default_value="300"),
        DeclareLaunchArgument("lower_raw", default_value="600"),
    ]

    velocity_guard = Node(
        package="leader_approach_control",
        executable="velocity_guard_node",
        namespace="leader",
        name="velocity_guard",
        output="screen",
        parameters=[
            config("guard_config"),
            {
                "guard_enabled_on_startup": False,
                # Reverse is only needed when the Leader backs up in transport.
                "allow_reverse": ParameterValue(
                    PythonExpression(["'", direction, "' == 'backward'"]), value_type=bool
                ),
            },
        ],
    )

    mission = Node(
        package="cooperative_mission",
        executable="leader_mission_node",
        namespace="leader",
        name="mission_coordinator",
        output="screen",
        parameters=[
            config("mission_config"),
            {
                "require_gripper": ParameterValue(config("gripper_enabled"), value_type=bool),
                "transport_direction": direction,
                "transport_speed": ParameterValue(config("transport_speed"), value_type=float),
                "transport_duration": ParameterValue(config("transport_duration"), value_type=float),
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
                "Leader mission stack: nothing moves until "
                "'ros2 service call /mission/start std_srvs/srv/Trigger'."
            )),
            _include(
                "rescue_robot_bringup",
                "camera_apriltag.launch.py",
                {
                    "enable_depth": "true",
                    "enable_infra": "false",
                    "enable_imu": "false",
                    "enable_approach": "true",
                    "final_target_distance": config("final_target_distance"),
                },
            ),
            _include(
                "leader_approach_control",
                "approach_controller.launch.py",
                {"controller_enabled_on_startup": "false"},
            ),
            velocity_guard,
            _include(
                "leader_command_selector",
                "command_selector.launch.py",
                {"source_mode": "MISSION"},
                condition=IfCondition(config("start_command_selector")),
            ),
            GroupAction(
                condition=IfCondition(config("use_stm32_bridge")),
                actions=[
                    _include(
                        "stm32_bridge",
                        "stm32_bridge.launch.py",
                        {
                            "transport": "i2c",
                            "i2c_device": config("i2c_device"),
                            "i2c_address": config("i2c_address"),
                            "i2c_write_enabled": config("i2c_write_enabled"),
                            "namespace": "leader",
                        },
                    )
                ],
            ),
            _include(
                "rescue_robot_tools",
                "dynamixel_orin.launch.py",
                {
                    "robot": "leader",
                    "port": config("gripper_port"),
                    "baudrate": config("gripper_baudrate"),
                    "rx64_speed": config("rx64_speed"),
                    "startup_pose_enabled": "false",
                    "startup_torque": "false",
                },
                condition=IfCondition(config("gripper_enabled")),
            ),
            mission,
        ]
    )
