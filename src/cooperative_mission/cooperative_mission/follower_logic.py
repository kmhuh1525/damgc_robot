"""ROS-independent Follower mission state machine.

The Follower never decides the mission on its own. It executes acknowledged
Leader commands (:mod:`protocol`) using the existing Follower pipeline:

* ``command_selector`` source ``COOPERATION`` carries this node's own
  reposition/search motion and, during transport, the Leader common velocity
  converted into the Follower frame;
* ``command_selector`` source ``APPROACH`` hands control to the existing
  AprilTag hybrid approach controller for the opposite object face;
* ``velocity_guard`` stays the final safety boundary and is enabled only in
  motion states.

Any loss of the Leader heartbeat while moving, any failed actuator/service
request and every timeout ends in FAULT: motion zero, selector STOP, guard
disabled, grippers holding.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .actions import Action, ActionKind, ActionTracker
from .motion import (
    ZERO,
    GripperParameters,
    ManeuverExecutor,
    ManeuverParameters,
    PlanarCommand,
    Pose2D,
    SearchController,
    SearchParameters,
    TagTracker,
    clamp,
    gripper_close,
    gripper_lift,
    gripper_lower,
    gripper_open,
    gripper_open_for_approach,
    is_finite,
    leader_to_follower,
    parse_segments,
)
from .protocol import CommandMessage, FollowerCommand, FollowerState, StatusMessage

# Follower states that command motion and therefore need the Leader heartbeat.
_MOTION_STATES = {
    FollowerState.REPOSITION,
    FollowerState.SEARCH,
    FollowerState.APPROACH,
    FollowerState.TRANSPORT_READY,
}

_RELEASABLE = {
    FollowerState.GRASPED,
    FollowerState.LIFT_READY,
    FollowerState.LIFTING,
    FollowerState.LIFTED,
    FollowerState.TRANSPORT_READY,
    FollowerState.HOLD,
    FollowerState.FAULT,
}


@dataclass(frozen=True)
class FollowerMissionConfig:
    target_tag_id: int = -1
    tag_acquire_time: float = 0.3
    search: SearchParameters = field(default_factory=SearchParameters)
    reposition_segments: Tuple[str, ...] = ()
    maneuver: ManeuverParameters = field(default_factory=ManeuverParameters)
    odom_timeout: float = 0.5
    reacquire_timeout: float = 3.0
    max_reacquire: int = 3
    approach_timeout: float = 150.0
    aligned_hold_time: float = 0.2
    # Follower profile defaults from follower_apriltag_drive.launch.py
    gripper: GripperParameters = field(
        default_factory=lambda: GripperParameters(
            open_raw=950, close_raw=350, lift_raw=300, lower_raw=600
        )
    )
    grasp_settle_time: float = 2.0
    lift_duration: float = 3.5
    release_lower_time: float = 3.5
    release_open_settle: float = 1.0
    leader_timeout: float = 1.0
    target_velocity_timeout: float = 0.25
    leader_facing_opposite: bool = True
    max_transport_linear: float = 0.10
    max_transport_angular: float = 0.30
    action_timeout: float = 2.0

    def validate(self) -> None:
        self.search.validate()
        self.maneuver.validate()
        self.gripper.validate()
        parse_segments(self.reposition_segments)
        positives = (
            self.tag_acquire_time, self.odom_timeout, self.reacquire_timeout,
            self.approach_timeout, self.grasp_settle_time, self.lift_duration,
            self.release_lower_time, self.release_open_settle,
            self.leader_timeout, self.target_velocity_timeout,
            self.max_transport_linear, self.max_transport_angular,
            self.action_timeout,
        )
        if not is_finite(*positives) or any(v <= 0.0 for v in positives):
            raise ValueError("Follower mission parameters must be finite and positive")
        if not is_finite(self.aligned_hold_time) or self.aligned_hold_time < 0.0:
            raise ValueError("aligned_hold_time must be finite and not negative")
        if self.max_reacquire < 0:
            raise ValueError("max_reacquire must not be negative")


@dataclass
class FollowerOutputs:
    cooperation_command: PlanarCommand
    actions: List[Action]
    status: StatusMessage
    status_changed: bool


class FollowerMission:
    """Deterministic Follower executor; call :meth:`update` at 50 Hz."""

    def __init__(self, config: FollowerMissionConfig) -> None:
        config.validate()
        self._c = config
        self._segments = parse_segments(config.reposition_segments)
        self._state = FollowerState.IDLE
        self._state_since = 0.0
        self._changed = True
        self._detail = "waiting for Leader APPROACH"
        self._ready = False
        self._ready_detail = "starting"
        self._session = ""
        self._last_seq = 0
        self._actions = ActionTracker(config.action_timeout)
        self._tags = TagTracker(config.target_tag_id, config.tag_acquire_time)
        self._search = SearchController(config.search)
        self._maneuver: Optional[ManeuverExecutor] = None
        self._pose: Optional[Pose2D] = None
        self._pose_time: Optional[float] = None
        self._alignment_state = ""
        self._aligned_since: Optional[float] = None
        self._alignment_time: Optional[float] = None
        self._approach_armed_at: Optional[float] = None
        self._leader_state = ""
        self._leader_time: Optional[float] = None
        self._target: Optional[PlanarCommand] = None
        self._target_time: Optional[float] = None
        self._approach_start = 0.0
        self._reacquire_count = 0
        self._arming_transport = False
        self._release_open_time: Optional[float] = None

    # ------------------------------------------------------------------ props
    @property
    def state(self) -> FollowerState:
        return self._state

    @property
    def detail(self) -> str:
        return self._detail

    # ----------------------------------------------------------------- inputs
    def set_ready(self, ready: bool, detail: str = "") -> None:
        if ready != self._ready or detail != self._ready_detail:
            self._changed = True
        self._ready = bool(ready)
        self._ready_detail = detail

    def on_tag_detected(self, detected: bool, now: float) -> None:
        self._tags.on_detected(detected, now)

    def on_tag_id(self, tag_id: int, now: float) -> None:
        self._tags.on_tag_id(tag_id, now)

    def on_alignment_state(self, text: str, now: float) -> None:
        state = str(text).strip().upper()
        if state == "ALIGNED":
            if self._alignment_state != "ALIGNED" or self._aligned_since is None:
                self._aligned_since = now
        else:
            self._aligned_since = None
        self._alignment_state = state
        self._alignment_time = now

    def _aligned_for(self, now: float, floor: float) -> bool:
        if self._alignment_state != "ALIGNED" or self._aligned_since is None:
            return False
        if self._alignment_time is None or self._alignment_time < floor:
            return False
        if now - self._alignment_time > 0.5:
            return False
        return now - max(self._aligned_since, floor) >= self._c.aligned_hold_time

    def on_odometry(self, x: float, y: float, yaw: float, now: float) -> None:
        if is_finite(x, y, yaw):
            self._pose = Pose2D(float(x), float(y), float(yaw))
            self._pose_time = now

    def on_leader_state(self, text: str, now: float) -> None:
        self._leader_state = str(text).strip().upper()
        self._leader_time = now

    def on_target_velocity(self, linear_x: float, angular_z: float, now: float) -> None:
        if not is_finite(linear_x, angular_z):
            self._target = None
            self._target_time = None
            return
        self._target = PlanarCommand(float(linear_x), float(angular_z))
        self._target_time = now

    def on_gripper_status(self, text: str, now: float) -> None:
        if str(text).startswith("ERROR") and self._state not in (
            FollowerState.IDLE, FollowerState.FAULT,
        ):
            self._fault(f"Follower gripper error: {text}", now)

    def on_action_result(self, action_id: int, success: bool, detail: str, now: float) -> None:
        action = self._actions.resolve(action_id)
        if action is None or success:
            return
        if action.is_safety_release or self._state == FollowerState.FAULT:
            self._detail = f"{action.kind.value} failed during stop: {detail}"
            self._changed = True
            return
        self._fault(f"{action.kind.value}={action.value} failed: {detail}", now)

    def on_command(self, message: CommandMessage, now: float) -> None:
        """Execute a Leader command exactly once per ``(session, seq)``."""
        self._changed = True  # always (re-)acknowledge promptly
        if message.session != self._session:
            self._session = message.session
            self._last_seq = 0
        if message.seq <= self._last_seq:
            return
        self._last_seq = message.seq
        self._handle(message.command, now)

    def take_actions(self) -> List[Action]:
        """Actions produced by :meth:`on_command`, for immediate execution.

        The node executes them before acknowledging so that, e.g., the
        Follower lift starts no later than the Leader's (which is triggered
        by that acknowledgement).
        """
        return self._actions.drain()

    # ----------------------------------------------------------------- update
    def update(self, now: float) -> FollowerOutputs:
        command = ZERO
        self._check_global(now)
        state = self._state
        if state == FollowerState.REPOSITION:
            command = self._update_reposition(now)
        elif state == FollowerState.SEARCH:
            command = self._update_search(now)
        elif state == FollowerState.APPROACH:
            self._update_approach(now)
        elif state == FollowerState.GRASP:
            if not self._actions.busy and now - self._state_since >= self._c.grasp_settle_time:
                self._enter(FollowerState.GRASPED, now, "holding the opposite face")
        elif state == FollowerState.LIFTING:
            if not self._actions.busy and now - self._state_since >= self._c.lift_duration:
                self._enter(FollowerState.LIFTED, now, "lift complete")
        elif state == FollowerState.LIFTED:
            if self._arming_transport and not self._actions.busy:
                self._arming_transport = False
                self._enter(FollowerState.TRANSPORT_READY, now,
                            "following the Leader common velocity")
        elif state == FollowerState.TRANSPORT_READY:
            command = self._transport_command(now)
        elif state == FollowerState.RELEASING:
            self._update_release(now)

        if self._state not in (
            FollowerState.REPOSITION, FollowerState.SEARCH, FollowerState.TRANSPORT_READY,
        ):
            command = ZERO
        changed, self._changed = self._changed, False
        return FollowerOutputs(
            cooperation_command=command,
            actions=self._actions.drain(),
            status=self.status(),
            status_changed=changed,
        )

    def status(self) -> StatusMessage:
        detail = self._detail if self._ready else f"{self._detail} | not ready: {self._ready_detail}"
        return StatusMessage(
            session=self._session,
            ack_seq=self._last_seq,
            state=self._state,
            ready=self._ready,
            detail=detail,
        )

    # --------------------------------------------------------------- commands
    def _handle(self, command: FollowerCommand, now: float) -> None:
        state = self._state
        if command == FollowerCommand.ABORT:
            if state not in (FollowerState.IDLE, FollowerState.FAULT):
                self._fault("Leader ABORT", now)
            return
        if command == FollowerCommand.RESET:
            self._actions.clear_waits()
            self._stop_motion(now)
            self._enter(FollowerState.IDLE, now, "reset; waiting for Leader APPROACH")
            return
        if command == FollowerCommand.APPROACH and state == FollowerState.IDLE:
            if not self._ready:
                self._fault(f"APPROACH received but not ready: {self._ready_detail}", now)
                return
            self._reacquire_count = 0
            self._approach_start = now
            self._actions.emit(ActionKind.GRIPPER, gripper_open_for_approach(self._c.gripper), now)
            self._actions.emit(ActionKind.SELECTOR_MODE, "COOPERATION", now)
            self._actions.emit(ActionKind.GUARD_ENABLE, True, now)
            if self._segments:
                self._maneuver = ManeuverExecutor(self._segments, self._c.maneuver)
                self._enter(FollowerState.REPOSITION, now, "moving to the opposite face")
            else:
                self._enter(FollowerState.SEARCH, now, "searching for the opposite-face tag")
            return
        if command == FollowerCommand.PREPARE_LIFT and state == FollowerState.GRASPED:
            self._enter(FollowerState.LIFT_READY, now, "lift armed")
            return
        if command == FollowerCommand.LIFT and state == FollowerState.LIFT_READY:
            self._actions.emit(ActionKind.GRIPPER, gripper_lift(self._c.gripper), now)
            self._enter(FollowerState.LIFTING, now, "lifting")
            return
        if command == FollowerCommand.PREPARE_TRANSPORT and state == FollowerState.LIFTED:
            self._target = None
            self._target_time = None
            self._actions.emit(ActionKind.SELECTOR_MODE, "COOPERATION", now)
            self._actions.emit(ActionKind.GUARD_ENABLE, True, now)
            self._arming_transport = True
            self._detail = "arming transport"
            return
        if command == FollowerCommand.END_TRANSPORT and state in (
            FollowerState.TRANSPORT_READY, FollowerState.LIFTED,
        ):
            self._arming_transport = False
            self._stop_motion(now, wait=True)
            self._enter(FollowerState.HOLD, now, "transport finished; holding")
            return
        if command == FollowerCommand.RELEASE and state in _RELEASABLE:
            self._actions.clear_waits()
            self._arming_transport = False
            self._stop_motion(now)
            self._actions.emit(ActionKind.GRIPPER, gripper_lower(self._c.gripper), now)
            self._release_open_time = None
            self._enter(FollowerState.RELEASING, now, "lowering the object")
            return
        self._detail = f"ignored {command.value} in {state.value}"

    # ---------------------------------------------------------------- phases
    def _check_global(self, now: float) -> None:
        expired = self._actions.expired(now)
        if expired is not None and self._state != FollowerState.FAULT:
            self._actions.resolve(expired.action_id)
            if not expired.is_safety_release:
                self._fault(f"no confirmation for {expired.kind.value}={expired.value}", now)
                return
        moving = self._state in _MOTION_STATES or self._arming_transport
        if moving:
            if self._leader_time is None or now - self._leader_time > self._c.leader_timeout:
                self._fault("Leader heartbeat (/mission/state) lost", now)
                return
            if self._leader_state == "FAULT":
                self._fault("Leader reported FAULT", now)

    def _update_reposition(self, now: float) -> PlanarCommand:
        if self._actions.busy:
            return ZERO
        if self._pose_time is None or now - self._pose_time > self._c.odom_timeout:
            self._fault("wheel odometry stale during reposition", now)
            return ZERO
        assert self._maneuver is not None
        command = self._maneuver.update(now, self._pose)
        if self._maneuver.failure is not None:
            self._fault(f"reposition failed: {self._maneuver.failure}", now)
            return ZERO
        if self._maneuver.done:
            self._enter(FollowerState.SEARCH, now, "reposition done; searching for tag")
            return ZERO
        return command

    def _update_search(self, now: float) -> PlanarCommand:
        if now - self._approach_start > self._c.approach_timeout:
            self._fault("Follower search/approach timed out", now)
            return ZERO
        if self._actions.busy:
            return ZERO
        if self._tags.acquired(now):
            self._actions.emit(ActionKind.SELECTOR_MODE, "APPROACH", now)
            self._actions.emit(ActionKind.APPROACH_ENABLE, True, now)
            self._approach_armed_at = None
            self._enter(FollowerState.APPROACH, now, "tag acquired; aligning to grasp pose")
            return ZERO
        if self._search.exhausted():
            self._fault("opposite-face tag not found after a full search sweep", now)
            return ZERO
        return self._search.update(now)

    def _update_approach(self, now: float) -> None:
        if self._actions.busy:
            self._approach_armed_at = None
        elif self._approach_armed_at is None:
            # The approach node resets its ALIGNED latch when the controller is
            # enabled; only trust ALIGNED samples after that confirmation.
            self._approach_armed_at = now
        if self._approach_armed_at is not None and self._aligned_for(now, self._approach_armed_at):
            self._actions.emit(ActionKind.APPROACH_ENABLE, False, now)
            self._actions.emit(ActionKind.SELECTOR_MODE, "STOP", now)
            self._actions.emit(ActionKind.GUARD_ENABLE, False, now)
            self._actions.emit(ActionKind.GRIPPER, gripper_close(self._c.gripper), now)
            self._enter(FollowerState.GRASP, now, "ALIGNED; closing Follower gripper")
            return
        if now - self._approach_start > self._c.approach_timeout:
            self._fault("Follower approach timed out", now)
            return
        if self._tags.lost_for(now) > self._c.reacquire_timeout:
            if self._reacquire_count >= self._c.max_reacquire:
                self._fault("tag lost repeatedly during approach", now)
                return
            self._reacquire_count += 1
            self._actions.emit(ActionKind.APPROACH_ENABLE, False, now)
            self._actions.emit(ActionKind.SELECTOR_MODE, "COOPERATION", now)
            self._enter(FollowerState.SEARCH, now,
                        f"tag lost; re-searching ({self._reacquire_count}/{self._c.max_reacquire})")

    def _transport_command(self, now: float) -> PlanarCommand:
        if (
            self._target is None
            or self._target_time is None
            or now - self._target_time > self._c.target_velocity_timeout
        ):
            return ZERO
        command = leader_to_follower(self._target, self._c.leader_facing_opposite)
        if (
            abs(command.linear_x) > self._c.max_transport_linear + 1.0e-9
            or abs(command.angular_z) > self._c.max_transport_angular + 1.0e-9
        ):
            self._fault("Leader transport velocity exceeds Follower limits", now)
            return ZERO
        return PlanarCommand(
            clamp(command.linear_x, self._c.max_transport_linear),
            clamp(command.angular_z, self._c.max_transport_angular),
        )

    def _update_release(self, now: float) -> None:
        if self._release_open_time is None:
            if now - self._state_since >= self._c.release_lower_time:
                self._actions.emit(ActionKind.GRIPPER, gripper_open(self._c.gripper), now)
                self._release_open_time = now
            return
        if not self._actions.busy and now - self._release_open_time >= self._c.release_open_settle:
            self._enter(FollowerState.IDLE, now, "released; waiting for Leader APPROACH")

    # --------------------------------------------------------------- helpers
    def _stop_motion(self, now: float, wait: bool = False) -> None:
        self._actions.emit(ActionKind.APPROACH_ENABLE, False, now, wait=wait)
        self._actions.emit(ActionKind.SELECTOR_MODE, "STOP", now, wait=wait)
        self._actions.emit(ActionKind.GUARD_ENABLE, False, now, wait=wait)

    def _fault(self, reason: str, now: float) -> None:
        if self._state == FollowerState.FAULT:
            return
        self._actions.clear_waits()
        self._arming_transport = False
        self._stop_motion(now)
        self._enter(FollowerState.FAULT, now, reason)

    def _enter(self, state: FollowerState, now: float, detail: str) -> None:
        if state == FollowerState.SEARCH:
            self._search.reset()
        self._state = state
        self._state_since = now
        self._detail = detail
        self._changed = True


def config_segments(items: Sequence[str]) -> Tuple[str, ...]:
    """Normalise a ROS string-array parameter (``['']`` means empty)."""
    return tuple(str(item) for item in items if str(item).strip())


__all__ = [
    "FollowerMission",
    "FollowerMissionConfig",
    "FollowerOutputs",
    "config_segments",
]
