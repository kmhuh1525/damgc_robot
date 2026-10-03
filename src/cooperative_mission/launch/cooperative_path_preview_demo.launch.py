"""Run the offline plan source, formation preview, and RViz instructions."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("cooperative_mission"), "config", "cooperative_path_preview.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="cooperative_mission",
                executable="cooperative_demo_plan_node",
                name="cooperative_demo_plan",
                output="screen",
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_object_path_planner_node",
                name="cooperative_object_path_planner",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="cooperative_mission",
                executable="cooperative_path_preview_node",
                name="cooperative_path_preview",
                output="screen",
                parameters=[config],
            ),
        ]
    )
