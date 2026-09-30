"""ROS-independent motion primitives shared by both mission coordinators."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PlanarCommand:
    """Differential-drive velocity command (base_link frame)."""

    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


ZERO = PlanarCommand()


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def is_finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


# --------------------------------------------------------------------------
# Cooperative transport velocity profile
# --------------------------------------------------------------------------


def trapezoid_speed(elapsed: float, duration: float, speed: float, acceleration: float) -> float:
    """Magnitude of a symmetric trapezoid that is non-zero only in ``[0, duration)``.

    The ramp-up and ramp-down are both contained inside ``duration``: at
    ``elapsed >= duration`` the command is exactly zero. When the requested
    speed cannot be reached within half the window the profile becomes a
    triangle with the same acceleration.
    """
    if not is_finite(elapsed, duration, speed, acceleration):
        return 0.0
    if duration <= 0.0 or speed <= 0.0 or acceleration <= 0.0:
        return 0.0
    if elapsed < 0.0 or elapsed >= duration:
        return 0.0
    ramp = min(speed / acceleration, duration / 2.0)
    peak = min(speed, acceleration * ramp)
    if elapsed < ramp:
        return acceleration * elapsed
    remaining = duration - elapsed
    if remaining < ramp:
        return acceleration * remaining
    return peak


def transport_command(
    elapsed: float,
    duration: float,
    speed: float,
    acceleration: float,
    direction_sign: float,
) -> PlanarCommand:
    """Straight-line common velocity in the Leader ``base_link`` frame."""
    magnitude = trapezoid_speed(elapsed, duration, speed, acceleration)
    if magnitude == 0.0:
        return ZERO
    return PlanarCommand(linear_x=math.copysign(magnitude, direction_sign), angular_z=0.0)


def leader_to_follower(command: PlanarCommand, facing_opposite: bool) -> PlanarCommand:
    """Convert the Leader-frame common velocity into the Follower base frame.

    With both robots rigidly holding opposite faces of one object they face
    each other: the same world translation is ``-linear_x`` for the Follower.
    (The existing ``leader_path_follower.py`` uses the same convention.)
    """
    if not facing_opposite:
        return command
    return PlanarCommand(linear_x=-command.linear_x, angular_z=command.angular_z)


# --------------------------------------------------------------------------
# Tag acquisition / loss tracking
# --------------------------------------------------------------------------


class TagTracker:
    """Debounce ``supply/detected`` + ``supply/tag_id`` into acquired/lost."""

    def __init__(self, target_tag_id: int, acquire_time: float) -> None:
        self._target_tag_id = int(target_tag_id)
        self._acquire_time = float(acquire_time)
        self._detected = False
        self._tag_id = -1
        self._visible_since: Optional[float] = None
        self._last_seen: Optional[float] = None

    def reset(self) -> None:
        self._visible_since = None
        self._last_seen = None

    def on_detected(self, detected: bool, now: float) -> None:
        self._detected = bool(detected)
        self._refresh(now)

    def on_tag_id(self, tag_id: int, now: float) -> None:
        self._tag_id = int(tag_id)
        self._refresh(now)

    def _matches(self) -> bool:
        if not self._detected or self._tag_id < 0:
            return False
        return self._target_tag_id < 0 or self._tag_id == self._target_tag_id

    def _refresh(self, now: float) -> None:
        if self._matches():
            if self._visible_since is None:
                self._visible_since = now
            self._last_seen = now
        else:
            if self._visible_since is not None:
                # It was visible until this very sample.
                self._last_seen = now
            self._visible_since = None

    def acquired(self, now: float) -> bool:
        return (
            self._visible_since is not None
            and now - self._visible_since >= self._acquire_time
        )

    def lost_for(self, now: float) -> float:
        """Seconds since the target was last seen (``inf`` if never)."""
        if self._matches():
            return 0.0
        if self._last_seen is None:
            return math.inf
        return max(0.0, now - self._last_seen)


# --------------------------------------------------------------------------
# In-place step-and-dwell tag search
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SearchParameters:
    angular_speed: float = 0.15
    direction: float = 1.0
    step_angle: float = math.radians(20.0)
    dwell_time: float = 0.6
    max_angle: float = math.radians(360.0)

    def validate(self) -> None:
        if not is_finite(self.angular_speed, self.direction, self.step_angle,
                         self.dwell_time, self.max_angle):
            raise ValueError("search parameters must be finite")
        if self.angular_speed <= 0.0 or self.step_angle <= 0.0:
            raise ValueError("search angular_speed and step_angle must be positive")
        if self.dwell_time < 0.0 or self.max_angle < 0.0:
            raise ValueError("search dwell_time and max_angle must not be negative")
        if self.direction == 0.0:
            raise ValueError("search direction must be +1 or -1")


class SearchController:
    """Rotate ``step_angle`` then pause ``dwell_time`` so the detector can settle.

    The swept angle is integrated from the commanded rate, so the search does
    not depend on odometry. ``exhausted`` becomes true after ``max_angle``.
    """

    def __init__(self, parameters: SearchParameters) -> None:
        parameters.validate()
        self._p = parameters
        self._swept = 0.0
        self._step_swept = 0.0
        self._dwell_until: Optional[float] = None
        self._last_time: Optional[float] = None

    def reset(self) -> None:
        self._swept = 0.0
        self._step_swept = 0.0
        self._dwell_until = None
        self._last_time = None

    @property
    def swept_angle(self) -> float:
        return self._swept

    def exhausted(self) -> bool:
        return self._swept >= self._p.max_angle - 1.0e-9

    def update(self, now: float) -> PlanarCommand:
        dt = 0.0 if self._last_time is None else max(0.0, min(now - self._last_time, 0.1))
        self._last_time = now
        if self.exhausted():
            return ZERO
        if self._dwell_until is not None:
            if now < self._dwell_until:
                return ZERO
            self._dwell_until = None
            self._step_swept = 0.0
            dt = 0.0
        increment = self._p.angular_speed * dt
        self._swept += increment
        self._step_swept += increment
        if self._step_swept >= self._p.step_angle:
            self._dwell_until = now + self._p.dwell_time
            return ZERO
        return PlanarCommand(0.0, math.copysign(self._p.angular_speed, self._p.direction))


# --------------------------------------------------------------------------
# Odometry-based relative maneuver (e.g. drive around the object)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Segment:
    kind: str  # "turn" (radians), "drive" (metres) or "wait" (seconds)
    value: float


def parse_segments(items: Sequence[str]) -> List[Segment]:
    """Parse ``["turn:90", "drive:0.6", "wait:0.5"]`` (degrees / metres / s)."""
    segments: List[Segment] = []
    for raw in items:
        text = str(raw).strip()
        if not text:
            continue
        kind, sep, value_text = text.partition(":")
        kind = kind.strip().lower()
        if not sep:
            raise ValueError(f"segment '{text}' must look like kind:value")
        try:
            value = float(value_text)
        except ValueError as error:
            raise ValueError(f"segment '{text}' has a non-numeric value") from error
        if not math.isfinite(value):
            raise ValueError(f"segment '{text}' is not finite")
        if kind == "turn":
            if abs(value) > 180.0:
                raise ValueError(
                    f"segment '{text}' turn must be within +/-180 deg; split larger turns"
                )
            segments.append(Segment("turn", math.radians(value)))
        elif kind == "drive":
            segments.append(Segment("drive", value))
        elif kind == "wait":
            if value < 0.0:
                raise ValueError(f"segment '{text}' wait must not be negative")
            segments.append(Segment("wait", value))
        else:
            raise ValueError(f"segment '{text}' kind must be turn, drive or wait")
    return segments


@dataclass(frozen=True)
class ManeuverParameters:
    drive_speed: float = 0.05
    min_drive_speed: float = 0.02
    drive_gain: float = 1.0
    drive_tolerance: float = 0.01
    turn_speed: float = 0.20
    min_turn_speed: float = 0.06
    turn_gain: float = 1.2
    turn_tolerance: float = math.radians(2.0)
    heading_gain: float = 1.0
    segment_time_margin: float = 5.0

    def validate(self) -> None:
        values = (
            self.drive_speed, self.min_drive_speed, self.drive_gain,
            self.drive_tolerance, self.turn_speed, self.min_turn_speed,
            self.turn_gain, self.turn_tolerance, self.heading_gain,
            self.segment_time_margin,
        )
        if not is_finite(*values) or any(v <= 0.0 for v in values):
            raise ValueError("maneuver parameters must be finite and positive")
        if self.min_drive_speed > self.drive_speed or self.min_turn_speed > self.turn_speed:
            raise ValueError("maneuver minimum speeds must not exceed maximum speeds")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class ManeuverExecutor:
    """Execute relative turn/drive/wait segments with wheel-odometry feedback."""

    def __init__(self, segments: Sequence[Segment], parameters: ManeuverParameters) -> None:
        parameters.validate()
        self._segments = list(segments)
        self._p = parameters
        self._index = 0
        self._start_pose: Optional[Pose2D] = None
        self._start_time: Optional[float] = None
        self.failure: Optional[str] = None

    @property
    def done(self) -> bool:
        return self._index >= len(self._segments)

    @property
    def index(self) -> int:
        return self._index

    def _deadline(self, segment: Segment) -> float:
        if segment.kind == "turn":
            nominal = abs(segment.value) / self._p.min_turn_speed
        elif segment.kind == "drive":
            nominal = abs(segment.value) / self._p.min_drive_speed
        else:
            nominal = segment.value
        return nominal + self._p.segment_time_margin

    def update(self, now: float, pose: Optional[Pose2D]) -> PlanarCommand:
        if self.done or self.failure is not None:
            return ZERO
        segment = self._segments[self._index]
        if self._start_time is None:
            self._start_time = now
            self._start_pose = pose
        if now - self._start_time > self._deadline(segment):
            self.failure = f"segment {self._index} ({segment.kind}) timed out"
            return ZERO

        if segment.kind == "wait":
            if now - self._start_time >= segment.value:
                self._advance()
            return ZERO

        if pose is None or self._start_pose is None:
            if self._start_pose is None and pose is not None:
                self._start_pose = pose
            return ZERO

        start = self._start_pose
        if segment.kind == "turn":
            turned = normalize_angle(pose.yaw - start.yaw)
            error = normalize_angle(segment.value - turned)
            if abs(error) <= self._p.turn_tolerance:
                self._advance()
                return ZERO
            rate = max(self._p.min_turn_speed, min(self._p.turn_speed, abs(error) * self._p.turn_gain))
            return PlanarCommand(0.0, math.copysign(rate, error))

        # drive: signed progress along the starting heading, heading hold.
        progress = (pose.x - start.x) * math.cos(start.yaw) + (pose.y - start.y) * math.sin(start.yaw)
        remaining = segment.value - progress
        if abs(remaining) <= self._p.drive_tolerance:
            self._advance()
            return ZERO
        speed = max(self._p.min_drive_speed, min(self._p.drive_speed, abs(remaining) * self._p.drive_gain))
        heading_error = normalize_angle(start.yaw - pose.yaw)
        angular = clamp(heading_error * self._p.heading_gain, self._p.turn_speed)
        return PlanarCommand(math.copysign(speed, remaining), angular)

    def _advance(self) -> None:
        self._index += 1
        self._start_pose = None
        self._start_time = None


# --------------------------------------------------------------------------
# Dynamixel (RX-64 lift / RX-28 jaw) raw command helpers
# --------------------------------------------------------------------------
#
# ``rescue_robot_tools/dynamixel_orin_node.py`` accepts
# ``[rx64_raw, rx28_raw, rx64_torque, rx28_torque]`` where -1 leaves a field
# unchanged. The helpers below never disable torque on the jaw so a closed
# gripper keeps holding the object while the lift arm moves.


@dataclass(frozen=True)
class GripperParameters:
    open_raw: int
    close_raw: int
    lift_raw: int
    lower_raw: int
    approach_rx64_raw: int = -1
    rx64_min: int = 260
    rx64_max: int = 670

    def validate(self) -> None:
        for name in ("open_raw", "close_raw"):
            value = getattr(self, name)
            if not 1 <= value <= 1021:
                raise ValueError(f"{name} must be between 1 and 1021")
        for name in ("lift_raw", "lower_raw"):
            value = getattr(self, name)
            if not self.rx64_min <= value <= self.rx64_max:
                raise ValueError(
                    f"{name} must be between {self.rx64_min} and {self.rx64_max}"
                )
        if self.approach_rx64_raw != -1 and not (
            self.rx64_min <= self.approach_rx64_raw <= self.rx64_max
        ):
            raise ValueError("approach_rx64_raw must be -1 or inside the RX-64 limits")


def gripper_open_for_approach(p: GripperParameters) -> Tuple[float, ...]:
    if p.approach_rx64_raw >= 0:
        return (float(p.approach_rx64_raw), float(p.open_raw), 1.0, 1.0)
    return (-1.0, float(p.open_raw), -1.0, 1.0)


def gripper_close(p: GripperParameters) -> Tuple[float, ...]:
    return (-1.0, float(p.close_raw), -1.0, 1.0)


def gripper_lift(p: GripperParameters) -> Tuple[float, ...]:
    return (float(p.lift_raw), -1.0, 1.0, -1.0)


def gripper_lower(p: GripperParameters) -> Tuple[float, ...]:
    return (float(p.lower_raw), -1.0, 1.0, -1.0)


def gripper_open(p: GripperParameters) -> Tuple[float, ...]:
    return (-1.0, float(p.open_raw), -1.0, 1.0)
