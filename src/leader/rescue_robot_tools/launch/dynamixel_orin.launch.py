from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("robot", default_value="leader"),
        DeclareLaunchArgument("rx64_speed", default_value="10"),
        DeclareLaunchArgument("rx28_speed", default_value="40"),
        DeclareLaunchArgument("lift_enabled", default_value="false"),
        DeclareLaunchArgument("startup_rx64_raw", default_value="600"),
        DeclareLaunchArgument("startup_rx28_raw", default_value="500"),
        DeclareLaunchArgument("startup_torque", default_value="true"),
        DeclareLaunchArgument("startup_pose_enabled", default_value="true"),
        Node(
            package="rescue_robot_tools",
            executable="dynamixel_orin_node.py",
            name="dynamixel_orin_node",
            namespace=LaunchConfiguration("robot"),
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("baudrate"), value_type=int
                ),
                "robot": LaunchConfiguration("robot"),
                "lift_enabled": ParameterValue(LaunchConfiguration("lift_enabled"), value_type=bool),
                "rx64_speed": ParameterValue(
                    LaunchConfiguration("rx64_speed"), value_type=int
                ),
                "rx28_speed": ParameterValue(
                    LaunchConfiguration("rx28_speed"), value_type=int
                ),
                "startup_rx64_raw": ParameterValue(
                    LaunchConfiguration("startup_rx64_raw"), value_type=int
                ),
                "startup_rx28_raw": ParameterValue(
                    LaunchConfiguration("startup_rx28_raw"), value_type=int
                ),
                "startup_torque": ParameterValue(
                    LaunchConfiguration("startup_torque"), value_type=bool
                ),
                "startup_pose_enabled": ParameterValue(
                    LaunchConfiguration("startup_pose_enabled"), value_type=bool
                ),
            }],
        ),
    ])
