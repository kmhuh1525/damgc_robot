"""ROS-independent hybrid alignment decisions for base-frame AprilTag samples."""

from dataclasses import dataclass
from enum import Enum
from math import atan2, cos, hypot, isfinite, radians, sin
from typing import Optional

from rescue_robot_apriltag.approach_logic import ApproachState


class ControlMode(str, Enum):
    """Internal control phases published without changing the public state API."""

    TAG_LOST = "TAG_LOST"
    COARSE_TRACK = "COARSE_TRACK"
    NEAR_ALIGN = "NEAR_ALIGN"
    RECENTER = "RECENTER"
    FINAL_YAW_ALIGN = "FINAL_YAW_ALIGN"
    FINAL_APPROACH = "FINAL_APPROACH"
    BLIND_FINAL_APPROACH = "BLIND_FINAL_APPROACH"
    STABILIZING = "STABILIZING"
    ALIGNED = "ALIGNED"
    TOO_CLOSE = "TOO_CLOSE"


@dataclass(frozen=True)
class BaseAlignmentThresholds:
    """Thresholds for visibility-first hybrid tag-normal alignment."""

    orientation_engage_distance: float
    orientation_disengage_distance: float
    turn_enter_error_deg: float
    turn_exit_error_deg: float
    tag_recenter_enter_deg: float
    tag_recenter_exit_deg: float
    near_normal_correction_limit_deg: float
    pre_align_position_tolerance: float
    final_position_tolerance: float
    final_yaw_tolerance_deg: float
    final_realign_yaw_error_deg: float
    stable_time: float
    sample_timeout: float
    aligned_confirm_samples: int = 3
    stabilizing_tag_loss_grace_sec: float = 0.20

    def validate(self) -> None:
        """Raise ``ValueError`` when thresholds cannot define safe hysteresis."""
        if not all(isfinite(value) for value in self.__dict__.values()):
            raise ValueError("All base alignment thresholds must be finite")
        if self.orientation_engage_distance <= 0.0:
            raise ValueError("orientation_engage_distance must be positive")
        if self.orientation_disengage_distance <= self.orientation_engage_distance:
            raise ValueError(
                "orientation_disengage_distance must exceed engage distance"
            )
        if not 0.0 <= self.turn_exit_error_deg < self.turn_enter_error_deg:
            raise ValueError("turn exit must be non-negative and below turn enter")
        if not 0.0 <= self.tag_recenter_exit_deg < self.tag_recenter_enter_deg:
            raise ValueError(
                "recenter exit must be non-negative and below recenter enter"
            )
        if self.near_normal_correction_limit_deg < 0.0:
            raise ValueError("near normal correction limit must not be negative")
        if self.pre_align_position_tolerance <= 0.0:
            raise ValueError("pre_align_position_tolerance must be positive")
        if self.final_position_tolerance <= 0.0:
            raise ValueError("final_position_tolerance must be positive")
        if self.final_yaw_tolerance_deg < 0.0:
            raise ValueError("final_yaw_tolerance_deg must not be negative")
        if self.final_realign_yaw_error_deg <= self.final_yaw_tolerance_deg:
            raise ValueError("final re-align error must exceed final yaw tolerance")
        if self.stable_time < 0.0:
            raise ValueError("stable_time must not be negative")
        if self.sample_timeout < 0.0:
            raise ValueError("sample_timeout must not be negative")
        if self.aligned_confirm_samples < 1:
            raise ValueError("aligned_confirm_samples must be at least 1")
        if self.stabilizing_tag_loss_grace_sec < 0.0:
            raise ValueError("stabilizing_tag_loss_grace_sec must not be negative")

    @property
    def turn_enter_error_rad(self) -> float:
        return radians(self.turn_enter_error_deg)

    @property
    def turn_exit_error_rad(self) -> float:
        return radians(self.turn_exit_error_deg)

    @property
    def tag_recenter_enter_rad(self) -> float:
        return radians(self.tag_recenter_enter_deg)

    @property
    def tag_recenter_exit_rad(self) -> float:
        return radians(self.tag_recenter_exit_deg)

    @property
    def near_normal_correction_limit_rad(self) -> float:
        return radians(self.near_normal_correction_limit_deg)

    @property
    def final_yaw_tolerance_rad(self) -> float:
        return radians(self.final_yaw_tolerance_deg)

    @property
    def final_realign_yaw_error_rad(self) -> float:
        return radians(self.final_realign_yaw_error_deg)


@dataclass(frozen=True)
class BaseAlignmentMeasurement:
    """One coherent tag-center and tag-normal geometry sample in ``base_link``."""

    tag_x: float
    tag_y: float
    prealign_x: float
    prealign_y: float
    final_x: float
    final_y: float
    final_yaw_error: float
    stamp_seconds: float


@dataclass(frozen=True)
class NearControl:
    """Visibility-weighted steering and forward scaling for NEAR alignment."""

    steering_error: float
    forward_scale: float
    normal_correction: float


@dataclass(frozen=True)
class AlignmentDecision:
    """Public state plus internal mode and the active planar control target."""

    state: ApproachState
    mode: ControlMode
    control_x: float = 0.0
    control_y: float = 0.0


def compute_blind_remaining_distance(
    last_valid_tag_x: float,
    final_target_distance: float,
    max_distance: float,
    zero_tolerance: float = 1.0e-3,
) -> Optional[float]:
    """Return one safe, forward-only blind distance, or ``None``."""
    values = (
        last_valid_tag_x,
        final_target_distance,
        max_distance,
        zero_tolerance,
    )
    if not all(isfinite(value) for value in values):
        return None
    if final_target_distance <= 0.0 or max_distance < 0.0 or zero_tolerance < 0.0:
        return None
    remaining = last_valid_tag_x - final_target_distance
    if remaining < -zero_tolerance or remaining > max_distance:
        return None
    return max(0.0, remaining)


def is_blind_final_approach_eligible(
    *,
    enabled: bool,
    phase: AlignmentDecision,
    last_valid_tag_x: float,
    last_valid_timestamp: float,
    now_seconds: float,
    last_valid_yaw_error: float,
    last_valid_cross_track: float,
    final_target_distance: float,
    activation_max_tag_x: float,
    max_distance: float,
    handoff_max_age: float,
    yaw_tolerance: float,
    cross_track_tolerance: float,
    odometry_valid: bool,
) -> bool:
    """Gate blind motion to a fresh, close, final visual alignment sample."""
    if not enabled or not odometry_valid:
        return False
    if phase.state != ApproachState.FINAL_APPROACH:
        return False
    if phase.mode != ControlMode.FINAL_APPROACH:
        return False
    values = (
        last_valid_tag_x,
        last_valid_timestamp,
        now_seconds,
        last_valid_yaw_error,
        last_valid_cross_track,
        activation_max_tag_x,
        handoff_max_age,
        yaw_tolerance,
        cross_track_tolerance,
    )
    if not all(isfinite(value) for value in values):
        return False
    if (
        activation_max_tag_x <= 0.0
        or handoff_max_age < 0.0
        or yaw_tolerance < 0.0
        or cross_track_tolerance < 0.0
    ):
        return False
    age = now_seconds - last_valid_timestamp
    if age < 0.0 or age > handoff_max_age:
        return False
    if last_valid_tag_x > activation_max_tag_x:
        return False
    if abs(last_valid_yaw_error) > yaw_tolerance:
        return False
    if abs(last_valid_cross_track) > cross_track_tolerance:
        return False
    return compute_blind_remaining_distance(
        last_valid_tag_x,
        final_target_distance,
        max_distance,
    ) is not None


def compute_forward_progress(
    start_x: float,
    start_y: float,
    start_yaw: float,
    current_x: float,
    current_y: float,
) -> Optional[float]:
    """Project odometry displacement onto the blind-start forward heading."""
    values = (start_x, start_y, start_yaw, current_x, current_y)
    if not all(isfinite(value) for value in values):
        return None
    dx = current_x - start_x
    dy = current_y - start_y
    progress = cos(start_yaw) * dx + sin(start_yaw) * dy
    return progress if isfinite(progress) else None


def validate_forward_odom_progress(
    start: tuple, previous: Optional[tuple], current: tuple
) -> Optional[float]:
    """Apply the existing blind-motion displacement checks to one odom sample."""
    progress = compute_forward_progress(
        start[0], start[1], start[2], current[0], current[1]
    )
    if progress is None or progress < -0.01:
        return None
    if previous is not None:
        step = hypot(current[0] - previous[0], current[1] - previous[1])
        total_dx = current[0] - start[0]
        total_dy = current[1] - start[1]
        lateral_deviation = abs(
            -sin(start[2]) * total_dx + cos(start[2]) * total_dy
        )
        yaw_deviation = abs(normalize_angle(current[2] - start[2]))
        if (
            not isfinite(step)
            or step > 0.05
            or lateral_deviation > 0.03
            or yaw_deviation > radians(12.0)
        ):
            return None
    return progress


def normalize_angle(angle: float) -> float:
    """Wrap a finite angle to ``[-pi, pi]``."""
    if not isfinite(angle):
        raise ValueError("Angle must be finite")
    return atan2(sin(angle), cos(angle))


def compute_near_control(
    tag_bearing: float,
    prealign_bearing: float,
    correction_limit: float,
    recenter_exit: float,
    recenter_enter: float,
) -> NearControl:
    """Anchor steering on tag center and add a visibility-limited normal bias."""
    values = (
        tag_bearing,
        prealign_bearing,
        correction_limit,
        recenter_exit,
        recenter_enter,
    )
    if not all(isfinite(value) for value in values):
        raise ValueError("Near-control inputs must be finite")
    if correction_limit < 0.0 or not 0.0 <= recenter_exit < recenter_enter:
        raise ValueError("Near-control limits are invalid")

    absolute_bearing = abs(tag_bearing)
    if absolute_bearing <= recenter_exit:
        visibility_scale = 1.0
    elif absolute_bearing >= recenter_enter:
        visibility_scale = 0.0
    else:
        visibility_scale = (
            (recenter_enter - absolute_bearing)
            / (recenter_enter - recenter_exit)
        )

    requested_correction = normalize_angle(prealign_bearing - tag_bearing)
    effective_limit = correction_limit * visibility_scale
    normal_correction = max(
        -effective_limit, min(effective_limit, requested_correction)
    )
    return NearControl(
        steering_error=normalize_angle(tag_bearing + normal_correction),
        forward_scale=visibility_scale,
        normal_correction=normal_correction,
    )


class BaseAlignmentStateMachine:
    """Hybrid FAR/NEAR/final state machine with visibility-first hysteresis."""

    def __init__(self, thresholds: BaseAlignmentThresholds) -> None:
        thresholds.validate()
        self._thresholds = thresholds
        self.reset()

    def reset(self) -> None:
        """Clear tag identity, phase latches, turn history, and stability."""
        self._stable_since: Optional[float] = None
        self._confirmation_count = 0
        self._stabilizing_loss_since: Optional[float] = None
        self._last_valid_aligned = False
        self._aligned_latched = False
        self._active_tag_id: Optional[int] = None
        self._orientation_engaged = False
        self._recenter_active = False
        self._turn_direction = 0
        self._final_phase = False
        self._final_approach_active = False

    def update(
        self,
        measurement: Optional[BaseAlignmentMeasurement],
        now_seconds: float,
        tag_id: Optional[int],
        is_new_observation: bool = True,
    ) -> AlignmentDecision:
        """Return a coherent state, internal mode, and control target."""
        if not isfinite(now_seconds):
            raise ValueError("now_seconds must be finite")

        # A changed selected tag is a new mission target and explicitly clears
        # the terminal latch.  A missing frame for the same target does not.
        if (
            tag_id is not None
            and self._active_tag_id is not None
            and tag_id != self._active_tag_id
        ):
            self.reset()
        if self._aligned_latched and tag_id in (None, self._active_tag_id):
            return AlignmentDecision(ApproachState.ALIGNED, ControlMode.ALIGNED)

        if not self._is_valid_sample(measurement, now_seconds, tag_id):
            if (
                measurement is None
                and self._stable_since is not None
                and self._last_valid_aligned
            ):
                if self._stabilizing_loss_since is None:
                    self._stabilizing_loss_since = now_seconds
                if (
                    now_seconds - self._stabilizing_loss_since
                    <= self._thresholds.stabilizing_tag_loss_grace_sec
                    + 1.0e-9
                ):
                    return AlignmentDecision(
                        ApproachState.STABILIZING, ControlMode.STABILIZING
                    )
            self.reset()
            return AlignmentDecision(ApproachState.TAG_LOST, ControlMode.TAG_LOST)
        assert measurement is not None
        assert tag_id is not None

        if tag_id != self._active_tag_id:
            self.reset()
            self._active_tag_id = tag_id

        if self._stabilizing_loss_since is not None:
            # Do not credit a dropout toward stable time.  The candidate is
            # resumed at the same elapsed visual-stability point.
            self._stable_since = (
                self._stable_since
                + now_seconds
                - self._stabilizing_loss_since
                if self._stable_since is not None
                else now_seconds
            )
            self._stabilizing_loss_since = None

        tag_range = hypot(measurement.tag_x, measurement.tag_y)
        tag_bearing = atan2(measurement.tag_y, measurement.tag_x)

        if not self._final_phase:
            if self._orientation_engaged:
                if tag_range > self._thresholds.orientation_disengage_distance:
                    self._orientation_engaged = False
                    self._recenter_active = False
            elif tag_range <= self._thresholds.orientation_engage_distance:
                self._orientation_engaged = True
                self._turn_direction = 0

        if not self._orientation_engaged and not self._final_phase:
            return self._coarse_decision(measurement, tag_bearing)

        if self._recenter_active:
            if abs(tag_bearing) <= self._thresholds.tag_recenter_exit_rad:
                self._recenter_active = False
            else:
                return self._recenter_decision(measurement, tag_bearing)
        elif abs(tag_bearing) >= self._thresholds.tag_recenter_enter_rad:
            self._recenter_active = True
            return self._recenter_decision(measurement, tag_bearing)

        if not self._final_phase:
            prealign_error = hypot(measurement.prealign_x, measurement.prealign_y)
            if prealign_error <= self._thresholds.pre_align_position_tolerance:
                self._final_phase = True
            elif measurement.prealign_x <= 0.0:
                if (
                    measurement.final_x
                    >= -self._thresholds.final_position_tolerance
                ):
                    self._final_phase = True
                else:
                    return self._leave_stable(
                        AlignmentDecision(
                            ApproachState.TOO_CLOSE, ControlMode.TOO_CLOSE
                        )
                    )
            else:
                prealign_bearing = atan2(
                    measurement.prealign_y, measurement.prealign_x
                )
                near = compute_near_control(
                    tag_bearing,
                    prealign_bearing,
                    self._thresholds.near_normal_correction_limit_rad,
                    self._thresholds.tag_recenter_exit_rad,
                    self._thresholds.tag_recenter_enter_rad,
                )
                control_range = prealign_error * near.forward_scale
                return self._leave_stable(
                    AlignmentDecision(
                        ApproachState.APPROACH,
                        ControlMode.NEAR_ALIGN,
                        control_range * cos(near.steering_error),
                        control_range * sin(near.steering_error),
                    )
                )

        return self._final_decision(measurement, now_seconds, is_new_observation)

    def _coarse_decision(
        self, measurement: BaseAlignmentMeasurement, tag_bearing: float
    ) -> AlignmentDecision:
        """Track only the tag center while outside orientation range."""
        if self._turn_direction and (
            self._turn_direction * tag_bearing
            <= self._thresholds.turn_exit_error_rad
        ):
            self._turn_direction = 0
        if not self._turn_direction:
            if tag_bearing >= self._thresholds.turn_enter_error_rad:
                self._turn_direction = 1
            elif tag_bearing <= -self._thresholds.turn_enter_error_rad:
                self._turn_direction = -1

        if self._turn_direction > 0:
            state = ApproachState.TURN_LEFT
        elif self._turn_direction < 0:
            state = ApproachState.TURN_RIGHT
        else:
            state = ApproachState.APPROACH
        return self._leave_stable(
            AlignmentDecision(
                state,
                ControlMode.COARSE_TRACK,
                measurement.tag_x,
                measurement.tag_y,
            )
        )

    def _recenter_decision(
        self, measurement: BaseAlignmentMeasurement, tag_bearing: float
    ) -> AlignmentDecision:
        """Stop forward motion and rotate using tag-center bearing only."""
        state = (
            ApproachState.TURN_LEFT
            if tag_bearing > 0.0
            else ApproachState.TURN_RIGHT
        )
        return self._leave_stable(
            AlignmentDecision(
                state,
                ControlMode.RECENTER,
                measurement.tag_x,
                measurement.tag_y,
            )
        )

    def _final_decision(
        self,
        measurement: BaseAlignmentMeasurement,
        now_seconds: float,
        is_new_observation: bool,
    ) -> AlignmentDecision:
        yaw_error = measurement.final_yaw_error
        tag_bearing = atan2(measurement.tag_y, measurement.tag_x)
        visibility = compute_near_control(
            tag_bearing,
            tag_bearing,
            0.0,
            self._thresholds.tag_recenter_exit_rad,
            self._thresholds.tag_recenter_enter_rad,
        )
        if self._final_approach_active:
            if abs(yaw_error) > self._thresholds.final_realign_yaw_error_rad:
                self._final_approach_active = False
        elif abs(yaw_error) <= self._thresholds.final_yaw_tolerance_rad:
            self._final_approach_active = True

        if not self._final_approach_active:
            state = (
                ApproachState.FINE_ALIGN_LEFT
                if yaw_error > 0.0
                else ApproachState.FINE_ALIGN_RIGHT
            )
            return self._leave_stable(
                AlignmentDecision(
                    state,
                    ControlMode.FINAL_YAW_ALIGN,
                    measurement.final_x,
                    measurement.final_y,
                )
            )

        final_error = hypot(measurement.final_x, measurement.final_y)
        if final_error > self._thresholds.final_position_tolerance:
            if measurement.final_x <= 0.0:
                return self._leave_stable(
                    AlignmentDecision(ApproachState.TOO_CLOSE, ControlMode.TOO_CLOSE)
                )
            return self._leave_stable(
                AlignmentDecision(
                    ApproachState.FINAL_APPROACH,
                    ControlMode.FINAL_APPROACH,
                    measurement.final_x * visibility.forward_scale,
                    measurement.final_y * visibility.forward_scale,
                )
            )

        if abs(yaw_error) > self._thresholds.final_yaw_tolerance_rad:
            return self._leave_stable(
                AlignmentDecision(
                    ApproachState.FINAL_APPROACH,
                    ControlMode.FINAL_APPROACH,
                    measurement.final_x * visibility.forward_scale,
                    measurement.final_y * visibility.forward_scale,
                )
            )

        if self._stable_since is None or now_seconds < self._stable_since:
            self._stable_since = now_seconds
        if now_seconds - self._stable_since >= self._thresholds.stable_time:
            if is_new_observation:
                self._confirmation_count += 1
            if self._confirmation_count >= self._thresholds.aligned_confirm_samples:
                self._aligned_latched = True
                return AlignmentDecision(ApproachState.ALIGNED, ControlMode.ALIGNED)
        self._last_valid_aligned = True
        return AlignmentDecision(
            ApproachState.STABILIZING,
            ControlMode.STABILIZING,
            measurement.final_x,
            measurement.final_y,
        )

    def _is_valid_sample(
        self,
        measurement: Optional[BaseAlignmentMeasurement],
        now_seconds: float,
        tag_id: Optional[int],
    ) -> bool:
        if measurement is None or tag_id is None or tag_id < 0:
            return False
        if not all(isfinite(value) for value in measurement.__dict__.values()):
            return False
        if hypot(measurement.tag_x, measurement.tag_y) <= 1.0e-6:
            return False
        if measurement.stamp_seconds <= 0.0:
            return False
        age = now_seconds - measurement.stamp_seconds
        return 0.0 <= age <= self._thresholds.sample_timeout + 1.0e-9

    def _leave_stable(self, decision: AlignmentDecision) -> AlignmentDecision:
        self._stable_since = None
        self._confirmation_count = 0
        self._stabilizing_loss_since = None
        self._last_valid_aligned = False
        return decision
