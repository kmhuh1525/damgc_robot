"""Static integration contracts for the cooperative mission launch."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_leader_guard_feeds_selector_mission_input() -> None:
    config = (PACKAGE_ROOT / "config/leader_velocity_guard_mission.yaml").read_text(
        encoding="utf-8"
    )
    assert "command_topic: /leader/mission/cmd_vel_raw" in config
    assert "safe_command_topic: /leader/mission/cmd_vel_safe" in config
    assert "safe_command_topic: /leader/cmd_vel" not in config


def test_leader_launch_owns_selector_in_mission_mode() -> None:
    launch = (PACKAGE_ROOT / "launch/leader_mission.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"leader_command_selector"' in launch
    assert '"source_mode": "MISSION"' in launch
    assert '"start_command_selector"' in launch


def test_follower_mission_explicitly_allows_opposite_facing_reverse() -> None:
    launch = (PACKAGE_ROOT / "launch/follower_mission.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"allow_reverse": "true"' in launch
