"""Peer plus optional follower drive plumbing. Never launches a gripper."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename)), launch_arguments=arguments.items())


def start(context):
    role = LaunchConfiguration('role').perform(context)
    command = '/leader/cooperation/cmd_vel' if role == 'leader' else '/follower/mission/cmd_vel'
    actions = []
    if role == 'follower' and LaunchConfiguration('follower_drive').perform(context) == 'true':
        config = os.path.join(get_package_share_directory('follower_command_selector'), 'config', 'command_selector.yaml')
        actions += [Node(package='follower_command_selector', executable='command_selector_node',
                         namespace='follower', name='command_selector', output='screen',
                         parameters=[config, {'source_mode':'STOP'}], remappings=[('cmd_vel',command)]),
                    include('follower_control','selected_velocity_guard.launch.py',{'guard_enabled_on_startup':'false','allow_reverse':'true'})]
        if LaunchConfiguration('use_stm32_bridge').perform(context) == 'true':
            actions.append(GroupAction(scoped=True, actions=[
                SetRemap(src='cmd_vel', dst='/follower/safe_cmd_vel'),
                include('stm32_bridge','stm32_bridge.launch.py',
                        {'namespace':'follower','transport':'i2c',
                         'i2c_device':LaunchConfiguration('i2c_device'),
                         'i2c_address':LaunchConfiguration('i2c_address'),
                         'i2c_write_enabled':LaunchConfiguration('i2c_write_enabled')})]))
    actions.append(Node(package='cooperative_transport',executable='transport_peer',
                        namespace=role, output='screen', parameters=[{'role':role, 'command_topic':command,
                        'motion_enabled':ParameterValue(LaunchConfiguration('motion_enabled'), value_type=bool)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('role',default_value='leader',choices=['leader','follower']),
        DeclareLaunchArgument('follower_drive',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('use_stm32_bridge',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('i2c_write_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('motion_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('i2c_device',default_value='/dev/i2c-7'),
        DeclareLaunchArgument('i2c_address',default_value='66'), OpaqueFunction(function=start)])
