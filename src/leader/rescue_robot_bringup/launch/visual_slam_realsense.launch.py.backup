from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    headless = os.environ.get("DAMGC_VSLAM_HEADLESS", "0") == "1"
    odom_tf = LaunchConfiguration("publish_odom_to_base_tf")
    map_tf = LaunchConfiguration("publish_map_to_odom_tf")
    jitter_threshold = LaunchConfiguration("image_jitter_threshold_ms")
    description_share = get_package_share_directory("rescue_robot_description")
    robot_description_path = os.path.join(
        description_share, "urdf", "rescue_robot.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_vslam",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    visual_slam = Node(
        package="isaac_ros_visual_slam",
        executable="isaac_ros_visual_slam",
        name="visual_slam_node",
        parameters=[{
            "enable_image_denoising": False,
            "rectified_images": True,
            # The connected device is D435 (not D435i), so use stereo-only VO.
            "enable_imu_fusion": False,
            "gyro_noise_density": 0.000244,
            "gyro_random_walk": 0.000019393,
            "accel_noise_density": 0.001862,
            "accel_random_walk": 0.003,
            "calibration_frequency": 200.0,
            "image_jitter_threshold_ms": ParameterValue(jitter_threshold, value_type=float),
            # The robot description connects base_link to the RealSense frames.
            "base_frame": "base_link",
            # The dual-EKF setup owns these links unless explicitly enabled.
            "publish_odom_to_base_tf": ParameterValue(odom_tf, value_type=bool),
            "publish_map_to_odom_tf": ParameterValue(map_tf, value_type=bool),
            # Debug renderings are optional; odometry/status and rosbag output
            # remain active in headless mode.
            "enable_slam_visualization": not headless,
            "enable_landmarks_view": not headless,
            "enable_observations_view": not headless,
            "camera_optical_frames": [
                "camera_infra1_optical_frame",
                "camera_infra2_optical_frame",
            ],
        }],
        remappings=[
            ("visual_slam/image_0", "/leader/camera/infra1/image_rect_raw"),
            ("visual_slam/camera_info_0", "/leader/camera/infra1/camera_info"),
            ("visual_slam/image_1", "/leader/camera/infra2/image_rect_raw"),
            ("visual_slam/camera_info_1", "/leader/camera/infra2/camera_info"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("publish_odom_to_base_tf", default_value="false"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
        DeclareLaunchArgument("image_jitter_threshold_ms", default_value="22.0"),
        robot_state_publisher,
        visual_slam,
    ])
