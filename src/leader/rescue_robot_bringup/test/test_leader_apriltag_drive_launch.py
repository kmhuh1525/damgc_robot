"""Static contracts for the integrated Leader AprilTag drive launch."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
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


def _load_camera():
    spec = importlib.util.spec_from_file_location(
        "camera_apriltag_launch", CAMERA_LAUNCH_FILE
    )
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
    assert 'DeclareLaunchArgument("gripper_close_raw", default_value="480")' in source
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


def test_shared_resource_switches_default_to_standalone_compatibility():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        description = module.generate_launch_description()

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()
    for name in (
        "start_camera",
        "start_camera_processing",
        "start_robot_state_publisher",
        "start_velocity_guard",
        "start_command_selector",
        "use_stm32_bridge",
    ):
        assert perform_substitutions(context, arguments[name].default_value) == "true"
    assert (
        perform_substitutions(context, arguments["selector_mode"].default_value)
        == "APPROACH"
    )


def test_shared_resource_switches_are_forwarded_or_conditioned():
    module = _load()
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        description = module.generate_launch_description()

    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    camera_include = next(
        entity for entity in includes
        if "enable_approach" in dict(entity.launch_arguments)
    )
    camera_arguments = dict(camera_include.launch_arguments)
    for name in (
        "start_camera",
        "start_camera_processing",
        "start_robot_state_publisher",
    ):
        context = LaunchContext()
        context.launch_configurations[name] = "false"
        assert camera_arguments[name].perform(context) == "false"

    guard_include = next(
        entity for entity in includes
        if "guard_enabled_on_startup" in dict(entity.launch_arguments)
    )
    bridge_include = next(
        entity for entity in includes
        if dict(entity.launch_arguments).get("namespace") == "leader"
    )
    selector_include = next(
        entity for entity in includes
        if "source_mode" in dict(entity.launch_arguments)
    )
    for name, include in (
        ("start_velocity_guard", guard_include),
        ("start_command_selector", selector_include),
        ("use_stm32_bridge", bridge_include),
    ):
        context = LaunchContext()
        context.launch_configurations[name] = "false"
        assert include.condition.evaluate(context) is False


def test_motion_sources_route_only_through_the_selector() -> None:
    teleop = (
        PACKAGE_ROOT / "scripts/arrow_key_teleop.py"
    ).read_text(encoding="utf-8")
    guard = (
        PACKAGE_ROOT.parent
        / "leader_approach_control/config/velocity_guard.yaml"
    ).read_text(encoding="utf-8")
    mapping_script = (
        PACKAGE_ROOT.parents[2] / "scripts/run_vslam_mapping.sh"
    ).read_text(encoding="utf-8")

    assert '"/leader/teleop/cmd_vel"' in teleop
    assert "safe_command_topic: /leader/approach/cmd_vel_safe" in guard
    assert "source_mode:=TELEOP" in mapping_script
    assert "-p command_topic:=/leader/teleop/cmd_vel" in mapping_script


def test_camera_launch_disables_only_shared_resource_owners():
    module = _load_camera()

    def package_share(package_name):
        if package_name == "rescue_robot_description":
            return str(PACKAGE_ROOT.parent / "rescue_robot_description")
        if package_name == "rescue_robot_apriltag":
            return str(PACKAGE_ROOT.parent / "rescue_robot_apriltag")
        return "/tmp/share"

    with patch.object(module, "get_package_share_directory", side_effect=package_share):
        description = module.generate_launch_description()

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    default_context = LaunchContext()
    for name in (
        "start_camera",
        "start_camera_processing",
        "start_robot_state_publisher",
    ):
        assert (
            perform_substitutions(default_context, arguments[name].default_value)
            == "true"
        )

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "start_camera": "false",
            "start_camera_processing": "false",
            "start_robot_state_publisher": "false",
            "enable_approach": "true",
        }
    )
    camera_include = next(
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    )
    assert camera_include.condition.evaluate(context) is False

    nodes = {
        entity.node_executable: entity
        for entity in description.entities
        if isinstance(entity, Node)
    }
    assert nodes["robot_state_publisher"].condition.evaluate(context) is False
    assert nodes["camera_info_qos_bridge.py"].condition.evaluate(context) is False
    assert nodes["rectify_node"].condition.evaluate(context) is False
    assert nodes["apriltag_node"].condition is None
    assert nodes["apriltag_approach_node"].condition.evaluate(context) is True


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
    assert perform_substitutions(context, arguments["post_align_grasp_target_distance"].default_value) == "0.20"
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
