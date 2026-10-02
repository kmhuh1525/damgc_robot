"""Start the ROS-side survivor pipeline using the verified child launches."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package_name, launch_file, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )
    # The child launches reuse names such as input_topic and output_topic.
    # Isolate their defaults so one child cannot configure another by accident.
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path)
            )
        ],
        scoped=True,
        forwarding=False,
        condition=condition,
    )


def generate_launch_description():
    """Launch the survivor ROS pipeline and its optional debug GUI.

    The detector uses the same Docker runtime as the existing wrapper because
    the verified YOLO runtime is installed in that image, not in the host ROS
    environment.  The wrapper keeps its interactive ``-it`` flags for manual
    fallback use; this launch omits only the TTY flag because launch itself is
    not a real terminal.  Do not run the wrapper separately while this launch
    is up.
    """
    repo_root = os.getcwd()
    model_cache = os.environ.get(
        "SURVIVOR_MODEL_CACHE",
        os.path.join(os.path.expanduser("~"), ".cache",
                     "damgc-survivor-ultralytics"),
    )
    os.makedirs(model_cache, exist_ok=True)
    detector_image = os.environ.get(
        "SURVIVOR_IMAGE", "damgc-survivor-yolo:humble"
    )
    detector_name = os.environ.get(
        "SURVIVOR_CONTAINER_NAME",
        f"damgc-survivor-detector-{os.getpid()}",
    )
    detector_command = [
        "docker", "run", "--rm", "-i",
        "--name", detector_name,
        "--runtime=nvidia",
        "--network", "host",
        "--ipc", "host",
        "-e", "NVIDIA_VISIBLE_DEVICES=all",
        "-e", "NVIDIA_DRIVER_CAPABILITIES=compute,utility",
        "-e", f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
        "-e", (
            "ROS_LOCALHOST_ONLY="
            f"{os.environ.get('ROS_LOCALHOST_ONLY', '0')}"
        ),
        "-e", (
            "RMW_IMPLEMENTATION="
            f"{os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')}"
        ),
        "-e", (
            "FASTDDS_BUILTIN_TRANSPORTS="
            f"{os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')}"
        ),
        "-e", "YOLO_CONFIG_DIR=/tmp",
        "-v", f"{model_cache}:/root/.cache/ultralytics",
        "-v", f"{repo_root}:/workspaces/isaac_ros-dev:ro",
        "--workdir", "/root/.cache/ultralytics",
        detector_image,
        "set +u; source /opt/ros/humble/setup.bash; "
        "source /opt/damgc_survivor_ws/install_survivor/setup.bash; "
        "set -u; ros2 launch rescue_robot_survivor person_detector.launch.py",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("enable_raw_visualizer", default_value="true"),
        DeclareLaunchArgument(
            "show_image_view",
            default_value="true",
            description=(
                "Start rqt_image_view for /leader/survivor/debug_image"
            ),
        ),
        _include(
            "rescue_robot_bringup", "survivor_camera_processing.launch.py"
        ),
        ExecuteProcess(
            cmd=detector_command,
            output="screen",
        ),
        _include("rescue_robot_survivor", "survivor_map_transform.launch.py"),
        _include(
            "rescue_robot_survivor",
            "survivor_map_visualizer.launch.py",
            condition=IfCondition(
                LaunchConfiguration("enable_raw_visualizer")
            ),
        ),
        _include("rescue_robot_survivor", "survivor_registry.launch.py"),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            arguments=["/leader/survivor/debug_image"],
            condition=IfCondition(LaunchConfiguration("show_image_view")),
            output="screen",
        ),
    ])
