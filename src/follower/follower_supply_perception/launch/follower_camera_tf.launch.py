"""Publish the measured Follower camera body and optical fixed transforms."""

from launch import LaunchDescription
from launch_ros.actions import Node

from follower_supply_perception.camera_extrinsic import FOLLOWER_CAMERA_TRANSFORMS


def generate_launch_description() -> LaunchDescription:
    """Create one static publisher for each child frame in the camera chain."""
    node_names = ("camera_mount_tf", "camera_optical_tf")
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace="/follower",
            name=node_name,
            arguments=list(transform.publisher_arguments()),
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
            output="screen",
        )
        for node_name, transform in zip(node_names, FOLLOWER_CAMERA_TRANSFORMS)
    ]
    return LaunchDescription(nodes)
