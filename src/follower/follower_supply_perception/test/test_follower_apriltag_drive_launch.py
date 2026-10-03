"""Static contracts for the integrated Follower AprilTag drive launch."""

import ast
import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_ROOT = PACKAGE_ROOT.parent
DRIVE_LAUNCH = PACKAGE_ROOT / "launch/follower_apriltag_drive.launch.py"
WHEEL_TEST_GUIDE = (
    PACKAGE_ROOT / "docs/FOLLOWER_WHEEL_DRIVE_TERMINAL_TEST.md"
)


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _string_constants(path: Path) -> list[str]:
    tree = ast.parse(_source(path), filename=str(path))
    return [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    ]


def test_integrated_launch_is_importable() -> None:
    assert DRIVE_LAUNCH.is_file()
    spec = importlib.util.spec_from_file_location(
        "follower_apriltag_drive_launch", DRIVE_LAUNCH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_integrated_launch_declares_bridge_arguments_and_safe_defaults() -> None:
    source = _source(DRIVE_LAUNCH)
    for argument in (
        "use_stm32_bridge",
        "i2c_device",
        "i2c_address",
        "i2c_write_enabled",
    ):
        assert 'DeclareLaunchArgument(\n            "%s"' % argument in source
    assert 'default_value="/dev/i2c-7"' in source
    assert 'default_value="66"' in source
    assert '"use_stm32_bridge",\n            default_value="true"' in source
    assert '"i2c_write_enabled",\n            default_value="true"' in source
    assert 'condition=IfCondition(use_stm32_bridge)' in source


def test_integrated_launch_reuses_exactly_one_selected_guard() -> None:
    launch_files = [
        value
        for value in _string_constants(DRIVE_LAUNCH)
        if value.endswith(".launch.py")
    ]
    assert launch_files.count("follower_apriltag.launch.py") == 1
    assert launch_files.count("approach_controller.launch.py") == 1
    assert launch_files.count("command_selector.launch.py") == 1
    assert launch_files.count("selected_velocity_guard.launch.py") == 1
    assert launch_files.count("velocity_guard.launch.py") == 0
    assert launch_files.count("stm32_bridge.launch.py") == 1


def test_integrated_startup_overrides_are_explicit() -> None:
    source = _source(DRIVE_LAUNCH)
    assert '{"enabled_on_startup": "true"}' in source
    assert '{"source_mode": "APPROACH"}' in source
    assert '{"guard_enabled_on_startup": "false"}' in source


def test_integrated_gripper_uses_follower_profile_and_alignment_values() -> None:
    source = _source(DRIVE_LAUNCH)
    assert 'DeclareLaunchArgument(\n            "gripper_enabled"' in source
    assert 'default_value="950"' in source
    assert 'default_value="350"' in source
    assert '"rx64_speed",\n            default_value="50"' in source
    assert '"lift_raw",\n            default_value="300"' in source
    assert '"lift_enabled",\n            default_value="true"' in source
    assert '"robot": "follower"' in source
    assert source.count('"robot": "follower"') == 2
    assert '"startup_pose_enabled": "false"' in source
    assert '"startup_torque": "false"' in source
    assert '"rx64_speed": LaunchConfiguration("rx64_speed")' in source
    assert '"tag_lost_idle_enabled": "false"' in source
    assert '"node_namespace": "follower"' in source
    assert '"detection_topic": "/follower/supply/detected"' in source
    assert '"alignment_topic": "/follower/base_alignment/state"' in source
    assert '"raw_command_topic": "/follower/dynamixel/command"' in source
    assert '"gripper_topic": "/follower/gripper/command"' in source


def test_gripper_master_gate_excludes_both_children() -> None:
    source = _source(DRIVE_LAUNCH)

    assert source.count(
        'condition=IfCondition(LaunchConfiguration("gripper_enabled"))'
    ) == 2
    assert source.count('"dynamixel_orin.launch.py"') == 1
    assert source.count('"gripper_sequence.launch.py"') == 1


def test_gripper_master_gate_condition_evaluates_false_and_true() -> None:
    spec = importlib.util.spec_from_file_location(
        "follower_apriltag_drive_conditions", DRIVE_LAUNCH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, "get_package_share_directory", return_value="/tmp/share"):
        description = module.generate_launch_description()

    conditioned_includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription) and entity.condition is not None
    ]
    assert len(conditioned_includes) == 2

    disabled_context = LaunchContext()
    disabled_context.launch_configurations["gripper_enabled"] = "false"
    assert all(
        not entity.condition.evaluate(disabled_context)
        for entity in conditioned_includes
    )

    enabled_context = LaunchContext()
    enabled_context.launch_configurations["gripper_enabled"] = "true"
    assert all(
        entity.condition.evaluate(enabled_context) for entity in conditioned_includes
    )


def test_selected_pipeline_and_bridge_remap_are_explicit() -> None:
    guard_launch = _source(
        FOLLOWER_ROOT / "follower_control/launch/selected_velocity_guard.launch.py"
    )
    source = _source(DRIVE_LAUNCH)
    assert '"command_topic": "/follower/selected_cmd_vel"' in guard_launch
    assert 'SetRemap(src="cmd_vel", dst="/follower/safe_cmd_vel")' in source
    assert '"namespace": "follower"' in source
    assert '"transport": "i2c"' in source


def test_individual_launch_defaults_remain_safe_and_compatible() -> None:
    approach_launch = _source(
        FOLLOWER_ROOT
        / "follower_approach_control/launch/approach_controller.launch.py"
    )
    selector_launch = _source(
        FOLLOWER_ROOT
        / "follower_command_selector/launch/command_selector.launch.py"
    )
    guard_launch = _source(
        FOLLOWER_ROOT / "follower_control/launch/selected_velocity_guard.launch.py"
    )
    assert '"enabled_on_startup",\n                default_value="false"' in approach_launch
    assert '"source_mode",\n                default_value="STOP"' in selector_launch
    assert (
        '"guard_enabled_on_startup",\n                default_value="false"'
        in guard_launch
    )


def test_atomic_command_and_blind_default_contracts() -> None:
    perception = _source(
        PACKAGE_ROOT / "follower_supply_perception/apriltag_approach_node.py"
    )
    controller = _source(
        FOLLOWER_ROOT
        / "follower_approach_control/follower_approach_control/approach_controller_node.py"
    )
    config = _source(PACKAGE_ROOT / "config/approach.yaml")
    message = _source(
        FOLLOWER_ROOT / "follower_alignment_msgs/msg/FollowerAlignmentCommand.msg"
    )
    assert 'FollowerAlignmentCommand, "alignment/command"' in perception
    assert 'FollowerAlignmentCommand,\n            "alignment/command"' in controller
    assert "blind_final_approach_enabled: false" in config
    assert "aligned_confirm_samples: 3" in config
    assert "base_stable_time: 0.30" in config
    assert "stabilizing_tag_loss_grace_sec: 0.30" in config
    assert "final_approach_tag_loss_grace_sec: 0.30" in config
    assert message.splitlines() == [
        "std_msgs/Header header",
        "geometry_msgs/Pose target_pose",
        "string control_mode",
        "string alignment_state",
    ]


def test_follower_runtime_code_and_config_have_no_leader_identifiers() -> None:
    runtime_files = []
    for package in FOLLOWER_ROOT.iterdir():
        if not package.is_dir():
            continue
        for path in package.rglob("*"):
            if not path.is_file() or any(
                part in {
                    "test",
                    "docs",
                    "__pycache__",
                    "follower_Leader Tracking",
                }
                for part in path.parts
            ):
                continue
            if path.suffix in {".py", ".yaml", ".xml", ".msg"}:
                runtime_files.append(path)
    forbidden = ("/leader/", "leader/tag", "leader_camera", "leader/odom")
    for path in runtime_files:
        lowered = _source(path).lower()
        for value in forbidden:
            assert value not in lowered, "%s leaked into %s" % (value, path)


def test_wheel_test_guide_preserves_the_guarded_runtime_path() -> None:
    guide = _source(WHEEL_TEST_GUIDE)
    required = (
        "use_stm32_bridge:=true",
        "i2c_write_enabled:=false",
        "i2c_write_enabled:=true",
        "/follower/cmd_vel",
        "/follower/selected_cmd_vel",
        "/follower/safe_cmd_vel",
        "/follower/velocity_guard/enable",
        "source_mode COOPERATION",
        "source_mode APPROACH",
        "source_mode STOP",
        "/follower/odom/raw",
        "blind_final_approach_enabled",
    )
    for value in required:
        assert value in guide
    assert "ros2 topic pub --rate 10 /follower/safe_cmd_vel" not in guide
