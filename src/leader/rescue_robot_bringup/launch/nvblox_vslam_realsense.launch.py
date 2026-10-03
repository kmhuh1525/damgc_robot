"""Map with the existing D435 stream and VSLAM as the odom TF source.

Use this when wheel odometry/local EKF is unavailable. Do not run it alongside
the dual-EKF launch, which also publishes odom -> base_link and map -> odom.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(name, arguments=None):
    path = os.path.join(os.path.dirname(__file__), name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    gxf_library_paths = [
        "/opt/ros/humble/lib",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization",
        "/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger",
    ]
    existing_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if existing_library_path:
        gxf_library_paths.append(existing_library_path)

    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", os.pathsep.join(gxf_library_paths)),
        _include("visual_slam_realsense.launch.py", {
            "publish_odom_to_base_tf": "true",
            "publish_map_to_odom_tf": "true",
            "image_jitter_threshold_ms": "50.0",
        }),
        _include("nvblox_realsense.launch.py"),
        _include("nvblox_nav2.launch.py"),
    ])
