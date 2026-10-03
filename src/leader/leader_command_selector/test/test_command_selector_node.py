"""ROS-message and source-switch tests for the Leader selector node."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

import leader_command_selector.command_selector_node as node_module
from leader_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
)
from leader_command_selector.command_selector_node import CommandSelectorNode


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def make_twist(linear_x: float = 0.1, angular_z: float = 0.2) -> Twist:
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def selector_parameters() -> SelectorParameters:
    return SelectorParameters(0.30, 0.35, 0.50, 0.35, 1.0e-9)


def command_harness(source: CommandSource) -> SimpleNamespace:
    sources = CommandSelectorNode._motion_sources()
    return SimpleNamespace(
        _source=source,
        _selector_parameters=selector_parameters(),
        _commands={item: None for item in sources},
        _received_seconds={item: None for item in sources},
        _command_pub=RecordingPublisher(),
        statuses=[],
        _publish_status=lambda status: None,
        get_logger=lambda: SimpleNamespace(warning=lambda *args, **kwargs: None),
    )


@pytest.mark.parametrize(
    ("selected", "incoming"),
    [
        (CommandSource.STOP, CommandSource.TELEOP),
        (CommandSource.TELEOP, CommandSource.APPROACH),
        (CommandSource.APPROACH, CommandSource.NAV2),
        (CommandSource.NAV2, CommandSource.TELEOP),
        (CommandSource.MISSION, CommandSource.APPROACH),
    ],
)
def test_unselected_source_is_completely_ignored(selected, incoming) -> None:
    harness = command_harness(selected)
    before = dict(harness._commands)
    CommandSelectorNode._on_command(harness, incoming, make_twist())
    assert harness._commands == before
    assert harness._command_pub.messages == []


@pytest.mark.parametrize(
    "source",
    [
        CommandSource.TELEOP,
        CommandSource.APPROACH,
        CommandSource.NAV2,
        CommandSource.MISSION,
    ],
)
def test_selected_source_is_cached_with_receipt_time(
    source, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 20.0)
    harness = command_harness(source)
    CommandSelectorNode._on_command(harness, source, make_twist(0.3, -0.4))
    assert harness._commands[source] == PlanarCommand(0.3, -0.4)
    assert harness._received_seconds[source] == 20.0


def parameter_harness() -> SimpleNamespace:
    sources = CommandSelectorNode._motion_sources()
    publisher = RecordingPublisher()
    statuses = []
    harness = SimpleNamespace(
        _source=CommandSource.TELEOP,
        _selector_parameters=selector_parameters(),
        _commands={item: PlanarCommand(0.5, 0.5) for item in sources},
        _received_seconds={item: 10.0 for item in sources},
        _command_pub=publisher,
        _publish_status=lambda status: statuses.append(status),
        _motion_sources=CommandSelectorNode._motion_sources,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        statuses=statuses,
    )
    harness._status_at = lambda now: CommandSelectorNode._status_at(harness, now)
    harness._selected_cache = lambda: CommandSelectorNode._selected_cache(harness)
    harness._clear_commands = lambda: CommandSelectorNode._clear_commands(harness)
    return harness


def test_source_change_publishes_zero_and_clears_every_cache(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 30.0)
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("source_mode", value="APPROACH")]
    )
    assert result.successful is True
    assert harness._source == CommandSource.APPROACH
    assert all(value is None for value in harness._commands.values())
    assert all(value is None for value in harness._received_seconds.values())
    assert len(harness._command_pub.messages) == 1
    assert harness._command_pub.messages[0].linear.x == 0.0
    assert harness.statuses[-1] == "WAITING_APPROACH"


@pytest.mark.parametrize("value", ["AUTO", "approach", 1])
def test_invalid_mode_is_rejected_without_changing_source(value) -> None:
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("source_mode", value=value)]
    )
    assert result.successful is False
    assert harness._source == CommandSource.TELEOP
    assert harness._command_pub.messages == []


def test_non_source_runtime_parameter_change_is_rejected() -> None:
    harness = parameter_harness()
    result = CommandSelectorNode._on_parameter_change(
        harness, [Parameter("teleop_timeout", value=1.0)]
    )
    assert result.successful is False


def test_shutdown_publishes_configured_zero_burst() -> None:
    publisher = RecordingPublisher()
    harness = SimpleNamespace(_shutdown_stop_count=3, _command_pub=publisher)
    CommandSelectorNode.stop(harness)
    assert len(publisher.messages) == 3
    assert all(message.linear.x == 0.0 for message in publisher.messages)


def test_twist_output_populates_only_planar_axes() -> None:
    message = CommandSelectorNode._to_twist(PlanarCommand(0.3, -0.4))
    assert message.linear.x == 0.3
    assert message.angular.z == -0.4
    assert message.linear.y == message.linear.z == 0.0
    assert message.angular.x == message.angular.y == 0.0
