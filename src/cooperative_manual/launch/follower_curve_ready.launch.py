"""Complete fresh follower stack; startup writes no gripper goal or torque."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def child(package,filename,arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package),'launch',filename])),launch_arguments=arguments.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_device',default_value='/dev/i2c-7'),
        DeclareLaunchArgument('gripper_port',default_value='/dev/ttyUSB0'),
        child('rescue_robot_tools','dynamixel_orin.launch.py',{
            'robot':'follower','port':LaunchConfiguration('gripper_port'),'rx28_speed':'40',
            'lift_enabled':'false','startup_pose_enabled':'false','startup_torque':'false',
            'shutdown_torque_off':'false'}),
        child('cooperative_manual','manual_curve.launch.py',{
            'role':'follower','motion_enabled':'true','drive_nodes':'true',
            'use_stm32_bridge':'true','i2c_write_enabled':'true',
            'i2c_device':LaunchConfiguration('i2c_device'),
            'gripper_enabled':'true','open_raw':'1000','close_raw':'331','gripper_settle':'9.0'}),
    ])
