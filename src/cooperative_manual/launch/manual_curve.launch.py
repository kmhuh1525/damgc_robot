"""Experimental geometry-aware actual-trail mode. Motion defaults OFF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keys={'role':'leader','motion_enabled':'false','drive_nodes':'false','use_stm32_bridge':'false','i2c_write_enabled':'false','command_topic':'','gripper_enabled':'false','close_raw':'','gripper_settle':''}
    arguments=[DeclareLaunchArgument(key,default_value=value) for key,value in keys.items()]
    included=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('cooperative_manual'),'launch','manual_straight.launch.py'])),launch_arguments={**{k:LaunchConfiguration(k) for k in keys},'peer_mode':'curve'}.items())
    return LaunchDescription(arguments+[included])
