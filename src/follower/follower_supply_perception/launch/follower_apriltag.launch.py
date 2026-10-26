"""Launch the complete follower USB camera and AprilTag perception pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the reproducible camera, rectification, tag, and approach pipeline."""
    video_device = LaunchConfiguration("video_device")
    camera_info_url = LaunchConfiguration("camera_info_url")
    apriltag_config = LaunchConfiguration("apriltag_config")
    approach_config = LaunchConfiguration("approach_config")

    package_share = FindPackageShare("follower_supply_perception")
    default_apriltag_config = PathJoinSubstitution(
        [package_share, "config", "apriltag.yaml"]
    )
    default_approach_config = PathJoinSubstitution(
        [package_share, "config", "approach.yaml"]
    )
    default_camera_info = [
        "file://",
        PathJoinSubstitution(
            [package_share, "config", "follower_usb_camera.yaml"]
        ),
    ]
    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "follower_camera_tf.launch.py"]
            )
        )
    )

    arguments = [
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="V4L2 USB camera device",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value=default_camera_info,
            description="Camera calibration URL used by usb_cam",
        ),
        DeclareLaunchArgument(
            "apriltag_config",
            default_value=default_apriltag_config,
            description="Absolute path to the apriltag_ros parameter YAML",
        ),
        DeclareLaunchArgument(
            "approach_config",
            default_value=default_approach_config,
            description="Absolute path to the approach-state parameter YAML",
        ),
    ]

    usb_camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="/follower/camera",
        name="usb_cam",
        output="screen",
        parameters=[
            {
                "video_device": video_device,
                # This camera exposes MJPEG 640x480 only at 120.101 FPS.  A
                # slower usb_cam publish timer leaves old mmap frames queued
                # and makes observation stamps progressively stale.  YUYV at
                # this resolution has a native 30 FPS mode, so acquisition and
                # publication remain paced without a source-frame backlog.
                "framerate": 30.0,
                "io_method": "mmap",
                "frame_id": "follower/follower_camera_optical_frame",
                # Keep the native YUYV payload through acquisition.  The
                # yuyv2rgb conversion could not sustain the device's native
                # 30 FPS on the Follower computer and accumulated stale mmap
                # frames before perception consumed them.
                "pixel_format": "yuyv",
                "av_device_format": "YUV422P",
                "image_width": 640,
                "image_height": 480,
                "camera_name": "follower_usb_camera",
                "camera_info_url": camera_info_url,
            }
        ],
    )

    rectify = Node(
        package="follower_supply_perception",
        executable="camera_pair_node",
        namespace="/follower",
        name="camera_pair",
        output="screen",
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="/follower/apriltag",
        name="apriltag",
        output="screen",
        parameters=[apriltag_config],
        remappings=[
            # Rectified mono image and calibration have an identical source stamp.
            ("image_rect", "/follower/tag_camera/image_rect"),
            ("camera_info", "/follower/tag_camera/camera_info"),
            ("tf", "/tf"),
        ],
    )

    approach = Node(
        package="follower_supply_perception",
        executable="apriltag_approach_node",
        namespace="/follower",
        name="apriltag_approach",
        output="screen",
        parameters=[approach_config],
    )

    return LaunchDescription(
        arguments + [camera_tf, usb_camera, rectify, apriltag, approach]
    )
