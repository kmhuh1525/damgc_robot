"""Configuration contracts for the Leader AprilTag velocity guard."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_guard_feeds_safe_approach_selector_input() -> None:
    config = (PACKAGE_ROOT / "config/velocity_guard.yaml").read_text(
        encoding="utf-8"
    )
    node = (
        PACKAGE_ROOT
        / "leader_approach_control/velocity_guard_node.py"
    ).read_text(encoding="utf-8")
    assert "command_topic: /leader/approach/cmd_vel_raw" in config
    assert "safe_command_topic: /leader/approach/cmd_vel_safe" in config
    assert '"safe_command_topic", "/leader/approach/cmd_vel_safe"' in node
    assert '"safe_command_topic", "/leader/cmd_vel"' not in node
