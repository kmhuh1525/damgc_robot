"""Tests for deterministic Leader command ownership and freshness logic."""

from math import inf, nan

import pytest

from leader_command_selector.command_selector_logic import (
    CommandSource,
    PlanarCommand,
    SelectorParameters,
    command_is_fresh,
    sanitize_command,
    select_command,
    selection_status,
)


def parameters(**overrides: float) -> SelectorParameters:
    values = {
        "teleop_timeout": 0.30,
        "approach_timeout": 0.35,
        "nav2_timeout": 0.50,
        "mission_timeout": 0.35,
        "axis_epsilon": 1.0e-9,
    }
    values.update(overrides)
    return SelectorParameters(**values)


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        (CommandSource.TELEOP, PlanarCommand(0.1, 0.2)),
        (CommandSource.APPROACH, PlanarCommand(0.03, -0.1)),
        (CommandSource.NAV2, PlanarCommand(0.2, 0.4)),
        (CommandSource.MISSION, PlanarCommand(0.05, 0.0)),
    ],
)
def test_each_fresh_source_is_forwarded(source, expected) -> None:
    assert select_command(source, 10.0, parameters(), expected, 9.9) == expected


def test_stop_is_zero_even_with_a_fresh_command() -> None:
    assert select_command(
        CommandSource.STOP,
        10.0,
        parameters(),
        PlanarCommand(9.0, 9.0),
        10.0,
    ) == PlanarCommand()


@pytest.mark.parametrize(
    ("source", "received"),
    [
        (CommandSource.TELEOP, 9.699),
        (CommandSource.APPROACH, 9.649),
        (CommandSource.NAV2, 9.499),
        (CommandSource.MISSION, 9.649),
    ],
)
def test_each_stale_source_fails_closed(source, received) -> None:
    assert select_command(
        source,
        10.0,
        parameters(),
        PlanarCommand(0.1, 0.2),
        received,
    ) == PlanarCommand()
    assert selection_status(
        source,
        10.0,
        parameters(),
        PlanarCommand(0.1, 0.2),
        received,
    ) == f"STALE_{source.value}"


def test_missing_selected_source_waits_at_zero() -> None:
    assert select_command(
        CommandSource.APPROACH, 10.0, parameters(), None, None
    ) == PlanarCommand()
    assert selection_status(
        CommandSource.APPROACH, 10.0, parameters(), None, None
    ) == "WAITING_APPROACH"


def test_status_reports_stop_and_active_source() -> None:
    assert selection_status(
        CommandSource.STOP, 10.0, parameters(), None, None
    ) == "STOP"
    assert selection_status(
        CommandSource.NAV2,
        10.0,
        parameters(),
        PlanarCommand(0.1, 0.0),
        9.9,
    ) == "ACTIVE_NAV2"


def test_freshness_includes_boundary_and_rejects_clock_rollback() -> None:
    assert command_is_fresh(10.30, 10.0, 0.30)
    assert not command_is_fresh(10.301, 10.0, 0.30)
    assert not command_is_fresh(9.9, 10.0, 0.30)
    assert not command_is_fresh(10.0, None, 0.30)


@pytest.mark.parametrize(
    "values",
    [
        (nan, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, inf),
        (0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.1, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.1, 0.0),
    ],
)
def test_invalid_or_nonplanar_input_is_rejected(values) -> None:
    assert sanitize_command(*values, axis_epsilon=1.0e-9) is None


def test_planar_input_is_preserved_without_clamping() -> None:
    assert sanitize_command(
        0.7, 0.0, 0.0, 0.0, 0.0, -1.2, 1.0e-9
    ) == PlanarCommand(0.7, -1.2)


@pytest.mark.parametrize(
    "invalid",
    [
        parameters(teleop_timeout=0.0),
        parameters(approach_timeout=-1.0),
        parameters(nav2_timeout=nan),
        parameters(mission_timeout=0.0),
        parameters(axis_epsilon=-1.0),
    ],
)
def test_invalid_parameters_are_rejected(invalid) -> None:
    with pytest.raises(ValueError):
        invalid.validate()
