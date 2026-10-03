import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    start_camera_processing = LaunchConfiguration("start_camera_processing")
    start_robot_state_publisher = LaunchConfiguration(
        "start_robot_state_publisher"
    )
    depth_enabled = LaunchConfiguration("enable_depth")
    sync_enabled = LaunchConfiguration("enable_sync")
    aligned_depth_enabled = LaunchConfiguration("align_depth.enable")
    infra_enabled = LaunchConfiguration("enable_infra")
    imu_enabled = LaunchConfiguration("enable_imu")
    approach_enabled = LaunchConfiguration("enable_approach")
    approach_config = LaunchConfiguration("approach_config")
    description_share = get_package_share_directory("rescue_robot_description")
    apriltag_share = get_package_share_directory("rescue_robot_apriltag")
    robot_description_path = os.path.join(description_share, "urdf", "rescue_robot.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Start the Leader RealSense driver. Set false to reuse the "
                "D435 owned by the mapping stack."
            ),
        ),
        DeclareLaunchArgument(
            "start_camera_processing",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Start the CameraInfo QoS bridge and RGB rectifier. Set false "
                "when the Survivor pipeline already owns these nodes."
            ),
        ),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Start the Leader robot_state_publisher. Set false when the "
                "VSLAM mapping stack already publishes the robot model TF."
            ),
        ),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_sync", default_value="true"),
        DeclareLaunchArgument("align_depth.enable", default_value="true"),
        DeclareLaunchArgument("enable_infra", default_value="false"),
        DeclareLaunchArgument("enable_imu", default_value="false"),
        DeclareLaunchArgument("enable_approach", default_value="false"),
        DeclareLaunchArgument(
            "approach_config",
            default_value=os.path.join(apriltag_share, "config", "approach.yaml"),
        ),
        DeclareLaunchArgument(
            "final_target_distance",
            default_value="0.23",
            description="Final tag-normal distance from tag plane to base_link",
        ),
        DeclareLaunchArgument("post_align_odom_enabled", default_value="false"),
        DeclareLaunchArgument("post_align_grasp_target_distance", default_value="0.16"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "camera_namespace": "leader",
                "camera_name": "camera",
                # A USB reset during startup can leave this D435 waiting indefinitely.
                "initial_reset": "false",
                "enable_color": "true",
                "enable_depth": depth_enabled,
                "enable_sync": sync_enabled,
                "align_depth.enable": aligned_depth_enabled,
                "enable_infra": infra_enabled,
                "enable_infra1": infra_enabled,
                "enable_infra2": infra_enabled,
                "enable_gyro": imu_enabled,
                "enable_accel": imu_enabled,
                "unite_imu_method": "2",
                "publish_tf": "true",
                "tf_publish_rate": "30.0",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "848x480x30",
            }.items(),
            condition=IfCondition(start_camera),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
            output="screen",
        ),
        Node(
            package="rescue_robot_apriltag",
            executable="camera_info_qos_bridge.py",
            name="camera_info_qos_bridge",
            condition=IfCondition(start_camera_processing),
            output="screen",
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            name="RectifyNode",
            remappings=[
                ("image", "/leader/camera/color/image_raw"),
                ("camera_info", "/leader/camera/color/camera_info_transient"),
                ("image_rect", "/leader/camera/color/image_rect"),
            ],
            parameters=[{"qos_overrides./camera_info.subscription.durability": "volatile"}],
            condition=IfCondition(start_camera_processing),
            output="screen",
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            namespace="leader/apriltag",
            name="apriltag",
            parameters=[os.path.join(apriltag_share, "config", "apriltag_leader.yaml")],
            remappings=[
                ("image_rect", "/leader/camera/color/image_rect"),
                ("camera_info", "/leader/camera/color/camera_info"),
            ],
            output="screen",
        ),
        Node(
            package="rescue_robot_apriltag",
            executable="apriltag_approach_node",
            namespace="leader",
            name="apriltag_approach",
            parameters=[
                approach_config,
                {
                    "final_target_distance": ParameterValue(
                        LaunchConfiguration("final_target_distance"),
                        value_type=float,
                    ),
                    "post_align_odom_enabled": ParameterValue(
                        LaunchConfiguration("post_align_odom_enabled"),
                        value_type=bool,
                    ),
                    "post_align_grasp_target_distance": ParameterValue(
                        LaunchConfiguration("post_align_grasp_target_distance"),
                        value_type=float,
                    ),
                },
            ],
            condition=IfCondition(approach_enabled),
            output="screen",
        ),
    ])
