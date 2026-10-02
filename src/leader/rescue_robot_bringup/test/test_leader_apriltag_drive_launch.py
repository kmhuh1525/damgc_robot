"""Static contracts for the integrated Leader AprilTag drive launch."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/leader_apriltag_drive.launch.py"
CAMERA_LAUNCH_FILE = PACKAGE_ROOT / "launch/camera_apriltag.launch.py"


def _load():
    spec = importlib.util.spec_from_file_location("leader_drive_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_integrated_launch_is_importable():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_integrated_launch_exposes_safe_gripper_and_lift_defaults():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("gripper_enabled", default_value="true")' in source
    assert 'DeclareLaunchArgument("rx64_speed", default_value="50")' in source
    assert 'DeclareLaunchArgument("gripper_open_raw", default_value="1000")' in source
    assert 'DeclareLaunchArgument("gripper_close_raw", default_value="450")' in source
    assert 'DeclareLaunchArgument("lift_enabled", default_value="true")' in source
    assert 'DeclareLaunchArgument("lift_raw", default_value="300")' in source
    assert 'DeclareLaunchArgument("gripper_lost_rx64_raw", default_value="600")' in source
    assert 'DeclareLaunchArgument("gripper_lost_rx28_raw", default_value="500")' in source
    assert source.count(
        'condition=IfCondition(LaunchConfiguration("gripper_enabled"))'
    ) == 2
    assert '"startup_pose_enabled": "false"' in source
    assert '"startup_torque": "false"' in source
    assert '"rx64_speed": LaunchConfiguration("rx64_speed")' in source
    assert '"tag_lost_idle_enabled": "false"' in source
    assert source.count('"dynamixel_orin.launch.py"') == 1
    assert source.count('"gripper_sequence.launch.py"') == 1
    assert '"alignment_topic": "/leader/base_alignment/state"' in source


def test_existing_drive_startup_safety_is_unchanged():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"guard_enabled_on_startup": "false"' in source
    assert '"controller_enabled_on_startup": "true"' in source
    assert '"transport": "i2c"' in source


def test_final_target_distance_override_is_forwarded_to_camera_launch():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        description = module.generate_launch_description()

    declaration = next(
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "final_target_distance"
    )
    default_context = LaunchContext()
    assert perform_substitutions(default_context, declaration.default_value) == "0.23"

    camera_include = next(
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and "enable_approach" in dict(entity.launch_arguments)
    )
    forwarded = dict(camera_include.launch_arguments)["final_target_distance"]
    override_context = LaunchContext()
    override_context.launch_configurations["final_target_distance"] = "0.19"
    assert forwarded.perform(override_context) == "0.19"


def test_camera_launch_applies_typed_final_distance_after_yaml_config():
    source = CAMERA_LAUNCH_FILE.read_text(encoding="utf-8")
    declaration = source.index('"final_target_distance",\n            default_value="0.23"')
    node = source.index('executable="apriltag_approach_node"')
    yaml_config = source.index("approach_config,", node)
    parameter_override = source.index(
        '"final_target_distance": ParameterValue(', yaml_config
    )

    assert declaration < node
    assert yaml_config < parameter_override
    assert 'LaunchConfiguration("final_target_distance")' in source[
        parameter_override:
    ]
    assert "value_type=float" in source[parameter_override:]


def test_post_align_launch_defaults_and_overrides_reach_approach_node():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        description = module.generate_launch_description()
    arguments = {
        entity.name: entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()
    assert perform_substitutions(context, arguments["post_align_odom_enabled"].default_value) == "true"
    assert perform_substitutions(context, arguments["post_align_grasp_target_distance"].default_value) == "0.16"
    camera_include = next(
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and "enable_approach" in dict(entity.launch_arguments)
    )
    forwarded = dict(camera_include.launch_arguments)
    context.launch_configurations["post_align_odom_enabled"] = "false"
    context.launch_configurations["post_align_grasp_target_distance"] = "0.18"
    assert forwarded["post_align_odom_enabled"].perform(context) == "false"
    assert forwarded["post_align_grasp_target_distance"].perform(context) == "0.18"
    camera_source = CAMERA_LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("post_align_odom_enabled", default_value="false")' in camera_source
    assert 'LaunchConfiguration("post_align_odom_enabled")' in camera_source
    assert 'LaunchConfiguration("post_align_grasp_target_distance")' in camera_source
    assert 'value_type=bool' in camera_source
