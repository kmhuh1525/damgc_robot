"""Add one manual peer; optional follower drive nodes must be sole owners."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def include(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', filename)), launch_arguments=arguments.items())


def create(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    role = get('role')
    command = get('command_topic') or ('/leader/cooperation/cmd_vel' if role == 'leader' else '/follower/mission/cmd_vel')
    actions = []
    if get('drive_nodes') == 'true':
        if role != 'follower':
            raise ValueError('leader uses existing mapping selector/bridge; drive_nodes only for follower')
        actions.append(Node(package='follower_command_selector', executable='command_selector_node', namespace='follower', name='command_selector', parameters=[{'source_mode':'STOP'}], remappings=[('cmd_vel',command)]))
        actions.append(include('follower_control','selected_velocity_guard.launch.py',{'guard_enabled_on_startup':'false','allow_reverse':'true'}))
        if get('use_stm32_bridge') == 'true':
            actions.append(GroupAction(scoped=True, actions=[SetRemap(src='cmd_vel',dst='/follower/safe_cmd_vel'), include('stm32_bridge','stm32_bridge.launch.py',{'namespace':'follower','transport':'i2c','i2c_device':get('i2c_device'),'i2c_address':'66','i2c_write_enabled':get('i2c_write_enabled')})]))
    elif get('use_stm32_bridge') == 'true':
        raise ValueError('use_stm32_bridge requires follower drive_nodes=true')
    actions.append(Node(package='cooperative_manual', executable='manual_straight', namespace=role, name='manual_straight', output='screen', parameters=[{'role':role,'motion_enabled':get('motion_enabled')=='true','command_topic':command,'speed':.03,'acceleration':.03,'timeout':.25,'gripper_enabled':get('gripper_enabled')=='true','open_raw':int(get('open_raw')),'close_raw':int(get('close_raw')) if get('close_raw') else (331 if role=='follower' else -1),'gripper_settle':float(get('gripper_settle')) if get('gripper_settle') else (9. if role=='follower' else 25.)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('role',default_value='leader',choices=['leader','follower']),
        DeclareLaunchArgument('motion_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('drive_nodes',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('use_stm32_bridge',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('i2c_write_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('i2c_device',default_value='/dev/i2c-7'),
        DeclareLaunchArgument('gripper_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('open_raw',default_value='1000'),
        DeclareLaunchArgument('close_raw',default_value=''),
        DeclareLaunchArgument('gripper_settle',default_value=''),
        DeclareLaunchArgument('command_topic',default_value=''), OpaqueFunction(function=create)])
