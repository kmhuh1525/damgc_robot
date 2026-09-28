"""Start Nav2 on the existing nvblox slice and odom -> base_link TF.

The controller output stays on /nav2/cmd_vel, away from the wheel bridge.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "config"))
    params_file = os.path.join(config_dir, "nvblox_nav2.yaml")
    tree_file = os.path.join(config_dir, "navigate_to_pose_nvblox.xml")
    through_tree_file = os.path.join(config_dir, "navigate_through_poses_nvblox.xml")
    nodes = [
        Node(
            package="rescue_robot_bringup",
            executable="costmap_topic_alias.py",
            name="costmap_topic_alias",
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "/nav2/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {
                "default_nav_to_pose_bt_xml": tree_file,
                "default_nav_through_poses_bt_xml": through_tree_file,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nvblox_nav2",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["planner_server", "controller_server", "bt_navigator"],
            }],
        ),
    ]
    return LaunchDescription(nodes)
