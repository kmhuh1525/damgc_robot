"""ROS-independent deterministic Leader command selection and validation."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional


class CommandSource(str, Enum):
    """The only velocity command owners accepted by the selector."""

    STOP = "STOP"
    TELEOP = "TELEOP"
    APPROACH = "APPROACH"
    NAV2 = "NAV2"
    MISSION = "MISSION"


@dataclass(frozen=True)
class PlanarCommand:
    """Validated differential-drive command."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class SelectorParameters:
    """Source-specific freshness and input-axis policy."""

    teleop_timeout: float
    approach_timeout: float
    nav2_timeout: float
    mission_timeout: float
    axis_epsilon: float

    def validate(self) -> None:
        """Reject timing or axis policies that cannot fail closed."""
        values = (
            self.teleop_timeout,
            self.approach_timeout,
            self.nav2_timeout,
            self.mission_timeout,
            self.axis_epsilon,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("Selector parameters must be finite")
        if min(
            self.teleop_timeout,
            self.approach_timeout,
            self.nav2_timeout,
            self.mission_timeout,
        ) <= 0.0:
            raise ValueError("Source timeouts must be greater than zero")
        if self.axis_epsilon < 0.0:
            raise ValueError("axis_epsilon must not be negative")

    def timeout_for(self, source: CommandSource) -> float:
        """Return the configured freshness timeout for a motion source."""
        if source == CommandSource.TELEOP:
            return self.teleop_timeout
        if source == CommandSource.APPROACH:
            return self.approach_timeout
        if source == CommandSource.NAV2:
            return self.nav2_timeout
        if source == CommandSource.MISSION:
            return self.mission_timeout
        raise ValueError("STOP has no source timeout")


def sanitize_command(
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    axis_epsilon: float,
) -> Optional[PlanarCommand]:
    """Return a finite planar command or ``None`` for malformed input."""
    values = (
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        axis_epsilon,
    )
    if not all(isfinite(value) for value in values) or axis_epsilon < 0.0:
        return None
    if any(
        abs(value) > axis_epsilon
        for value in (linear_y, linear_z, angular_x, angular_y)
    ):
        return None
    return PlanarCommand(float(linear_x), float(angular_z))


def command_is_fresh(
    now_seconds: float,
    received_seconds: Optional[float],
    timeout: float,
) -> bool:
    """Check a headerless Twist using its local monotonic receipt time."""
    if received_seconds is None:
        return False
    values = (now_seconds, received_seconds, timeout)
    if timeout <= 0.0 or not all(isfinite(value) for value in values):
        return False
    age = now_seconds - received_seconds
    return 0.0 <= age <= timeout + 1.0e-9


def select_command(
    source: CommandSource,
    now_seconds: float,
    parameters: SelectorParameters,
    command: Optional[PlanarCommand],
    received_seconds: Optional[float],
) -> PlanarCommand:
    """Return the explicitly selected fresh command, otherwise zero."""
    parameters.validate()
    if source == CommandSource.STOP:
        return PlanarCommand()
    if command is not None and command_is_fresh(
        now_seconds,
        received_seconds,
        parameters.timeout_for(source),
    ):
        return command
    return PlanarCommand()


def selection_status(
    source: CommandSource,
    now_seconds: float,
    parameters: SelectorParameters,
    command: Optional[PlanarCommand],
    received_seconds: Optional[float],
) -> str:
    """Describe STOP, waiting, active, or stale selector state."""
    parameters.validate()
    if source == CommandSource.STOP:
        return "STOP"
    if command is None or received_seconds is None:
        return f"WAITING_{source.value}"
    if command_is_fresh(
        now_seconds,
        received_seconds,
        parameters.timeout_for(source),
    ):
        return f"ACTIVE_{source.value}"
    return f"STALE_{source.value}"
