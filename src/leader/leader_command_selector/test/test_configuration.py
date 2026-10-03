"""Installed configuration contracts for the Leader selector."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_config_defaults_to_stop_with_source_timeouts() -> None:
    config = (PACKAGE_ROOT / "config/command_selector.yaml").read_text(
        encoding="utf-8"
    )
    assert "source_mode: STOP" in config
    assert "teleop_timeout: 0.30" in config
    assert "approach_timeout: 0.35" in config
    assert "nav2_timeout: 0.50" in config
    assert "mission_timeout: 0.35" in config


def test_node_owns_only_final_output_and_expected_inputs() -> None:
    node = (
        PACKAGE_ROOT
        / "leader_command_selector/command_selector_node.py"
    ).read_text(encoding="utf-8")
    assert '"teleop/cmd_vel"' in node
    assert '"approach/cmd_vel_safe"' in node
    assert '"/nav2/cmd_vel"' in node
    assert '"mission/cmd_vel_safe"' in node
    assert 'create_publisher(Twist, "cmd_vel"' in node
    assert '"command_selector/status"' in node


def test_launch_exposes_all_modes_with_generic_stop_default() -> None:
    launch = (PACKAGE_ROOT / "launch/command_selector.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'default_value="STOP"' in launch
    assert '["STOP", "TELEOP", "APPROACH", "NAV2", "MISSION"]' in launch
