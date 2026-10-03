"""Installed-mode and parameter contract tests for the Follower guard."""

import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def yaml_value(key: str) -> str:
    """Read one scalar from the compact guard YAML without an extra dependency."""
    pattern = re.compile(r"^\s*%s:\s*([^#\s]+)" % re.escape(key))
    config = PACKAGE_ROOT / "config/velocity_guard.yaml"
    for line in config.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            return match.group(1)
    raise AssertionError("missing YAML key %s" % key)


def test_legacy_topics_and_watchdog_are_preserved() -> None:
    assert yaml_value("command_timeout") == "0.3"
    assert yaml_value("command_topic") == "/follower/cmd_vel"
    assert yaml_value("safe_command_topic") == "/follower/safe_cmd_vel"


def test_guard_starts_disabled_with_reverse_rejected() -> None:
    assert yaml_value("guard_enabled_on_startup") == "false"
    assert yaml_value("allow_reverse") == "false"


def test_integrated_launch_overrides_only_the_input_topic() -> None:
    launch = (
        PACKAGE_ROOT / "launch/selected_velocity_guard.launch.py"
    ).read_text(encoding="utf-8")
    assert '"command_topic": "/follower/selected_cmd_vel"' in launch
    assert '"allow_reverse"' in launch
    assert 'default_value="false"' in launch
    assert "velocity_guard.yaml" in launch
    assert "/follower/safe_cmd_vel" not in launch
