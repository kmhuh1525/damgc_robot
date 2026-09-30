"""ROS-independent Leader mission state machine.

Scenario (project description):

1. Leader recognises the AprilTag ("QR code") on the object   LEADER_SEARCH/APPROACH
2. Leader grasps the object                                    LEADER_GRASP
3. Follower goes to the opposite face and grasps it            FOLLOWER_APPROACH
4. Both robots lift the object simultaneously                  LIFT_PREPARE/LIFT
5. Both robots drive straight in the same direction for 1 s    TRANSPORT_PREPARE/TRANSPORT

The Leader is the only authority: it sequences the Follower through
acknowledged commands (see :mod:`protocol`) and streams the common transport
velocity. Every wait has a timeout that ends in FAULT, and FAULT always zeroes
both robots' motion while the grippers keep holding the object.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from .actions import Action, ActionKind, ActionTracker
from .motion import (
    ZERO,
    GripperParameters,
    PlanarCommand,
    SearchController,
    SearchParameters,
    TagTracker,
    gripper_close,
    gripper_lift,
    gripper_lower,
    gripper_open,
    gripper_open_for_approach,
    is_finite,
    transport_command,
)
from .protocol import CommandMessage, FollowerCommand, FollowerState, StatusMessage


class LeaderState(str, Enum):
    IDLE = "IDLE"
    LEADER_SEARCH = "LEADER_SEARCH"
    LEADER_APPROACH = "LEADER_APPROACH"
    LEADER_GRASP = "LEADER_GRASP"
    FOLLOWER_APPROACH = "FOLLOWER_APPROACH"
    LIFT_PREPARE = "LIFT_PREPARE"
    LIFT = "LIFT"
    TRANSPORT_PREPARE = "TRANSPORT_PREPARE"
    TRANSPORT = "TRANSPORT"
    TRANSPORT_FINISH = "TRANSPORT_FINISH"
    DONE = "DONE"
    RELEASE = "RELEASE"
    FAULT = "FAULT"


# /cooperation/state values (Plan.md 5.6) derived from the mission state.
COOPERATION_STATE = {
    LeaderState.IDLE: "IDLE",
    LeaderState.LEADER_SEARCH: "LEADER_GRASPING",
    LeaderState.LEADER_APPROACH: "LEADER_GRASPING",
    LeaderState.LEADER_GRASP: "LEADER_GRASPING",
    LeaderState.FOLLOWER_APPROACH: "WAITING_FOLLOWER",
    LeaderState.LIFT_PREPARE: "LIFTING",
    LeaderState.LIFT: "LIFTING",
    LeaderState.TRANSPORT_PREPARE: "COOPERATING",
    LeaderState.TRANSPORT: "COOPERATING",
    LeaderState.TRANSPORT_FINISH: "COOPERATING",
    LeaderState.DONE: "HOLDING",
    LeaderState.RELEASE: "RELEASING",
    LeaderState.FAULT: "FAULT",
}

# States in which a fresh Follower status is mandatory.
_FOLLOWER_MONITORED = {
    LeaderState.FOLLOWER_APPROACH,
    LeaderState.LIFT_PREPARE,
    LeaderState.LIFT,
    LeaderState.TRANSPORT_PREPARE,
    LeaderState.TRANSPORT,
    LeaderState.TRANSPORT_FINISH,
    LeaderState.RELEASE,
}


@dataclass(frozen=True)
class LeaderMissionConfig:
    # Perception / search
    target_tag_id: int = -1
    tag_acquire_time: float = 0.3
    search: SearchParameters = field(default_factory=SearchParameters)
    reacquire_timeout: float = 3.0
    max_reacquire: int = 3
    approach_timeout: float = 120.0
    approach_command_timeout: float = 0.3
    aligned_hold_time: float = 0.2
    # Gripper (Leader profile defaults from leader_apriltag_drive.launch.py)
    gripper: GripperParameters = field(
        default_factory=lambda: GripperParameters(
            open_raw=1000, close_raw=450, lift_raw=300, lower_raw=600
        )
    )
    grasp_settle_time: float = 2.0
    lift_duration: float = 3.5
    lift_ack_margin: float = 3.0
    release_lower_time: float = 3.5
    release_open_settle: float = 1.0
    # Follower coordination
    require_follower: bool = True
    follower_status_timeout: float = 1.0
    follower_approach_timeout: float = 180.0
    handshake_timeout: float = 5.0
    command_resend_period: float = 0.2
    # Cooperative transport (Leader base_link frame)
    transport_direction: str = "forward"
    transport_speed: float = 0.05
    transport_acceleration: float = 0.20
    transport_duration: float = 1.0
    transport_leader_delay: float = 0.02
    transport_settle_time: float = 0.5
    transport_status_timeout: float = 0.35
    max_transport_speed: float = 0.10
    # Service / actuator confirmations
    action_timeout: float = 2.0

    @property
    def direction_sign(self) -> float:
        return 1.0 if self.transport_direction == "forward" else -1.0

    def validate(self) -> None:
        self.search.validate()
        self.gripper.validate()
        positives = (
            self.tag_acquire_time, self.reacquire_timeout, self.approach_timeout,
            self.approach_command_timeout, self.grasp_settle_time,
            self.lift_duration, self.lift_ack_margin, self.release_lower_time,
            self.release_open_settle, self.follower_status_timeout,
            self.follower_approach_timeout, self.handshake_timeout,
            self.command_resend_period, self.transport_speed,
            self.transport_acceleration, self.transport_duration,
            self.transport_settle_time, self.transport_status_timeout,
            self.max_transport_speed, self.action_timeout,
        )
        if not is_finite(*positives) or any(v <= 0.0 for v in positives):
            raise ValueError("mission timing/speed parameters must be finite and positive")
        if not is_finite(self.transport_leader_delay, self.aligned_hold_time):
            raise ValueError("transport_leader_delay/aligned_hold_time must be finite")
        if not 0.0 <= self.transport_leader_delay <= 0.2:
            raise ValueError("transport_leader_delay must be within 0..0.2 s")
        if self.aligned_hold_time < 0.0:
            raise ValueError("aligned_hold_time must not be negative")
        if self.max_reacquire < 0:
            raise ValueError("max_reacquire must not be negative")
        if self.transport_direction not in ("forward", "backward"):
            raise ValueError("transport_direction must be 'forward' or 'backward'")
        if self.transport_speed > self.max_transport_speed:
            raise ValueError("transport_speed exceeds max_transport_speed")


@dataclass
class LeaderOutputs:
    drive: PlanarCommand
    target_velocity: PlanarCommand
    actions: List[Action]
    follower_message: Optional[CommandMessage]
    state: LeaderState
    state_changed: bool
    detail: str


class LeaderMission:
    """Deterministic Leader coordinator; call :meth:`update` at 50 Hz."""

    def __init__(self, config: LeaderMissionConfig, session: str) -> None:
        config.validate()
        if not session:
            raise ValueError("session must not be empty")
        self._c = config
        self._session = session
        self._state = LeaderState.IDLE
        self._state_since = 0.0
        self._state_changed = True
        self._detail = "waiting for /mission/start"
        self._actions = ActionTracker(config.action_timeout)
        self._tags = TagTracker(config.target_tag_id, config.tag_acquire_time)
        self._search = SearchController(config.search)
        self._alignment_state = ""
        self._aligned_since: Optional[float] = None
        self._alignment_time: Optional[float] = None
        self._approach_cmd: Optional[PlanarCommand] = None
        self._approach_cmd_time: Optional[float] = None
        self._follower: Optional[StatusMessage] = None
        self._follower_time: Optional[float] = None
        self._seq = 0
        self._pending: Optional[CommandMessage] = None
        self._pending_last_sent: Optional[float] = None
        self._mission_start = 0.0
        self._reacquire_count = 0
        self._leader_lift_time: Optional[float] = None
        self._transport_start: Optional[float] = None
        self._release_lower_time: Optional[float] = None
        self._release_open_time: Optional[float] = None

    # ------------------------------------------------------------------ props
    @property
    def state(self) -> LeaderState:
        return self._state

    @property
    def session(self) -> str:
        return self._session

    @property
    def detail(self) -> str:
        return self._detail

    @property
    def follower_status(self) -> Optional[StatusMessage]:
        return self._follower

    # ----------------------------------------------------------------- inputs
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
        """ALIGNED seen in a sample after ``floor`` and held for ``aligned_hold_time``.

        A robot that is already latched ALIGNED when the phase starts (it has
        not moved since) counts from ``floor`` instead of being ignored.
        """
        if self._alignment_state != "ALIGNED" or self._aligned_since is None:
            return False
        if self._alignment_time is None or self._alignment_time < floor:
            return False
        if now - self._alignment_time > 0.5:  # perception stopped publishing
            return False
        return now - max(self._aligned_since, floor) >= self._c.aligned_hold_time

    def on_approach_command(self, linear_x: float, angular_z: float, now: float) -> None:
        if is_finite(linear_x, angular_z):
            self._approach_cmd = PlanarCommand(float(linear_x), float(angular_z))
            self._approach_cmd_time = now

    def on_follower_status(self, status: StatusMessage, now: float) -> None:
        self._follower = status
        self._follower_time = now

    def on_gripper_status(self, text: str, now: float) -> None:
        if str(text).startswith("ERROR") and self._state not in (
            LeaderState.IDLE, LeaderState.FAULT, LeaderState.DONE,
        ):
            self._fault(f"Leader gripper error: {text}", now)

    def on_action_result(self, action_id: int, success: bool, detail: str, now: float) -> None:
        action = self._actions.resolve(action_id)
        if action is None or success:
            return
        if action.is_safety_release or self._state == LeaderState.FAULT:
            self._detail = f"{action.kind.value} failed during stop: {detail}"
            return
        self._fault(f"{action.kind.value}={action.value} failed: {detail}", now)

    # --------------------------------------------------------------- requests
    def start(self, now: float, ready: bool = True, not_ready_reason: str = "") -> Tuple[bool, str]:
        if self._state != LeaderState.IDLE:
            return False, f"mission already active in {self._state.value}; abort/reset first"
        if not ready:
            return False, f"Leader not ready: {not_ready_reason}"
        if self._c.require_follower:
            follower = self._follower
            if follower is None or not self._follower_fresh(now, self._c.follower_status_timeout):
                return False, "Follower mission status not received; start the Follower first"
            if follower.state != FollowerState.IDLE:
                return False, f"Follower is {follower.state.value}, expected IDLE (call /mission/reset)"
            if not follower.ready:
                return False, f"Follower not ready: {follower.detail}"
        self._mission_start = now
        self._reacquire_count = 0
        self._actions.clear_waits()
        self._actions.emit(ActionKind.GRIPPER, gripper_open_for_approach(self._c.gripper), now)
        self._actions.emit(ActionKind.APPROACH_ENABLE, True, now)
        self._actions.emit(ActionKind.GUARD_ENABLE, True, now)
        self._enter(LeaderState.LEADER_SEARCH, now, "searching for the object tag")
        return True, "mission started"

    def abort(self, now: float, reason: str = "operator abort") -> Tuple[bool, str]:
        if self._state == LeaderState.IDLE:
            return True, "already idle"
        self._fault(reason, now)
        return True, "mission aborted; motion stopped, grippers keep holding"

    def reset(self, now: float) -> Tuple[bool, str]:
        if self._state not in (LeaderState.IDLE, LeaderState.FAULT, LeaderState.DONE):
            return False, f"cannot reset while {self._state.value}; abort first"
        self._actions.clear_waits()
        self._actions.emit(ActionKind.APPROACH_ENABLE, False, now, wait=False)
        self._actions.emit(ActionKind.GUARD_ENABLE, False, now, wait=False)
        self._send(FollowerCommand.RESET)
        self._enter(LeaderState.IDLE, now, "reset; waiting for /mission/start")
        return True, "reset; Follower RESET sent (grippers were not moved)"

    def release(self, now: float) -> Tuple[bool, str]:
        if self._state not in (LeaderState.DONE, LeaderState.FAULT):
            return False, f"release is allowed in DONE or FAULT, not {self._state.value}"
        if not self._follower_fresh(now, self._c.follower_status_timeout):
            return False, "Follower status is stale; cannot lower the object in sync"
        self._actions.clear_waits()
        self._release_lower_time = None
        self._release_open_time = None
        self._send(FollowerCommand.RELEASE)
        self._enter(LeaderState.RELEASE, now, "lowering the object together")
        return True, "release started"

    # ----------------------------------------------------------------- update
    def update(self, now: float) -> LeaderOutputs:
        drive = ZERO
        target = ZERO
        self._check_global(now)
        state = self._state

        if state == LeaderState.LEADER_SEARCH:
            drive = self._update_search(now)
        elif state == LeaderState.LEADER_APPROACH:
            drive = self._update_approach(now)
        elif state == LeaderState.LEADER_GRASP:
            if not self._actions.busy and now - self._state_since >= self._c.grasp_settle_time:
                self._send(FollowerCommand.APPROACH)
                self._enter(LeaderState.FOLLOWER_APPROACH, now,
                            "Leader holds the object; Follower approaching opposite face")
        elif state == LeaderState.FOLLOWER_APPROACH:
            if self._acked_in(FollowerState.GRASPED):
                self._send(FollowerCommand.PREPARE_LIFT)
                self._enter(LeaderState.LIFT_PREPARE, now, "both grasped; arming lift")
            elif now - self._state_since > self._c.follower_approach_timeout:
                self._fault("Follower did not grasp before follower_approach_timeout", now)
            elif self._follower is not None:
                self._detail = f"Follower {self._follower.state.value}: {self._follower.detail}"
        elif state == LeaderState.LIFT_PREPARE:
            if self._acked_in(FollowerState.LIFT_READY):
                self._send(FollowerCommand.LIFT)
                self._leader_lift_time = None
                self._enter(LeaderState.LIFT, now, "LIFT sent; lifting on Follower acknowledgement")
            elif now - self._state_since > self._c.handshake_timeout:
                self._fault("Follower did not arm the lift", now)
        elif state == LeaderState.LIFT:
            self._update_lift(now)
        elif state == LeaderState.TRANSPORT_PREPARE:
            if self._acked_in(FollowerState.TRANSPORT_READY) and not self._actions.busy:
                self._transport_start = now
                self._enter(LeaderState.TRANSPORT, now, "cooperative straight transport")
            elif now - self._state_since > self._c.handshake_timeout:
                self._fault("transport arming timed out", now)
        if self._state == LeaderState.TRANSPORT:
            drive, target = self._update_transport(now)
        elif self._state == LeaderState.TRANSPORT_FINISH:
            if self._acked_in(FollowerState.HOLD):
                self._enter(LeaderState.DONE, now,
                            "transport complete; both robots stopped and holding the object")
            elif now - self._state_since > self._c.handshake_timeout:
                self._fault("Follower did not confirm END_TRANSPORT", now)
        elif self._state == LeaderState.RELEASE:
            self._update_release(now)

        if self._state in (LeaderState.FAULT, LeaderState.IDLE, LeaderState.DONE):
            drive = ZERO
            target = ZERO
        return LeaderOutputs(
            drive=drive,
            target_velocity=target,
            actions=self._actions.drain(),
            follower_message=self._follower_message(now),
            state=self._state,
            state_changed=self._consume_changed(),
            detail=self._detail,
        )

    # --------------------------------------------------------------- helpers
    def _check_global(self, now: float) -> None:
        expired = self._actions.expired(now)
        if expired is not None and self._state != LeaderState.FAULT:
            self._actions.resolve(expired.action_id)
            if not expired.is_safety_release:
                self._fault(f"no confirmation for {expired.kind.value}={expired.value}", now)
                return
        if self._state in _FOLLOWER_MONITORED:
            timeout = (
                self._c.transport_status_timeout
                if self._state == LeaderState.TRANSPORT
                else self._c.follower_status_timeout
            )
            if not self._follower_fresh(now, timeout):
                self._fault("Follower status timeout", now)
            elif (
                self._follower is not None
                and self._follower.state == FollowerState.FAULT
                and self._acked()
            ):
                self._fault(f"Follower FAULT: {self._follower.detail}", now)

    def _update_search(self, now: float) -> PlanarCommand:
        if now - self._mission_start > self._c.approach_timeout:
            self._fault("Leader tag search/approach timed out", now)
            return ZERO
        if self._actions.busy:
            return ZERO
        if self._tags.acquired(now):
            self._enter(LeaderState.LEADER_APPROACH, now, "tag acquired; aligning to grasp pose")
            return ZERO
        if self._search.exhausted():
            self._fault("object tag not found after a full search sweep", now)
            return ZERO
        return self._search.update(now)

    def _update_approach(self, now: float) -> PlanarCommand:
        if not self._actions.busy and self._aligned_for(now, self._state_since):
            self._actions.emit(ActionKind.APPROACH_ENABLE, False, now)
            self._actions.emit(ActionKind.GUARD_ENABLE, False, now)
            self._actions.emit(ActionKind.GRIPPER, gripper_close(self._c.gripper), now)
            self._enter(LeaderState.LEADER_GRASP, now, "ALIGNED; closing Leader gripper")
            return ZERO
        if now - self._mission_start > self._c.approach_timeout:
            self._fault("Leader approach timed out", now)
            return ZERO
        if self._tags.lost_for(now) > self._c.reacquire_timeout:
            if self._reacquire_count >= self._c.max_reacquire:
                self._fault("tag lost repeatedly during approach", now)
                return ZERO
            self._reacquire_count += 1
            self._enter(LeaderState.LEADER_SEARCH, now,
                        f"tag lost; re-searching ({self._reacquire_count}/{self._c.max_reacquire})")
            return ZERO
        command = self._approach_cmd
        if (
            command is None
            or self._approach_cmd_time is None
            or now - self._approach_cmd_time > self._c.approach_command_timeout
            or command.linear_x < 0.0
        ):
            return ZERO
        return command

    def _update_lift(self, now: float) -> None:
        if self._leader_lift_time is None:
            follower = self._follower
            if self._acked() and follower is not None and follower.state in (
                FollowerState.LIFTING, FollowerState.LIFTED,
            ):
                # The Follower lifts on receipt; lifting here, on its
                # acknowledgement, keeps the two arms within one DDS hop and
                # guarantees the Leader never lifts alone.
                self._actions.emit(ActionKind.GRIPPER, gripper_lift(self._c.gripper), now)
                self._leader_lift_time = now
                self._detail = "both lift arms moving"
            elif now - self._state_since > self._c.handshake_timeout:
                self._fault("Follower did not acknowledge LIFT", now)
            return
        elapsed = now - self._leader_lift_time
        if (
            elapsed >= self._c.lift_duration
            and not self._actions.busy
            and self._follower is not None
            and self._follower.state == FollowerState.LIFTED
        ):
            self._send(FollowerCommand.PREPARE_TRANSPORT)
            self._actions.emit(ActionKind.GUARD_ENABLE, True, now)
            self._enter(LeaderState.TRANSPORT_PREPARE, now, "object lifted; arming transport")
        elif elapsed > self._c.lift_duration + self._c.lift_ack_margin:
            self._fault("Follower did not report LIFTED", now)

    def _update_transport(self, now: float) -> Tuple[PlanarCommand, PlanarCommand]:
        follower = self._follower
        if follower is None or follower.state != FollowerState.TRANSPORT_READY:
            self._fault("Follower left TRANSPORT_READY during transport", now)
            return ZERO, ZERO
        assert self._transport_start is not None
        elapsed = now - self._transport_start
        c = self._c
        target = transport_command(
            elapsed, c.transport_duration, c.transport_speed,
            c.transport_acceleration, c.direction_sign,
        )
        drive = transport_command(
            elapsed - c.transport_leader_delay, c.transport_duration,
            c.transport_speed, c.transport_acceleration, c.direction_sign,
        )
        if elapsed >= c.transport_duration + c.transport_leader_delay + c.transport_settle_time:
            self._send(FollowerCommand.END_TRANSPORT)
            self._actions.emit(ActionKind.GUARD_ENABLE, False, now)
            self._enter(LeaderState.TRANSPORT_FINISH, now, "transport window finished; stopping")
            return ZERO, ZERO
        return drive, target

    def _update_release(self, now: float) -> None:
        follower = self._follower
        if self._release_lower_time is None:
            if self._acked() and follower is not None and follower.state in (
                FollowerState.RELEASING, FollowerState.IDLE,
            ):
                self._actions.emit(ActionKind.GRIPPER, gripper_lower(self._c.gripper), now)
                self._release_lower_time = now
            elif now - self._state_since > self._c.handshake_timeout:
                self._fault("Follower did not acknowledge RELEASE", now)
            return
        if self._release_open_time is None:
            if now - self._release_lower_time >= self._c.release_lower_time:
                self._actions.emit(ActionKind.GRIPPER, gripper_open(self._c.gripper), now)
                self._release_open_time = now
            return
        if (
            now - self._release_open_time >= self._c.release_open_settle
            and not self._actions.busy
            and follower is not None
            and follower.state == FollowerState.IDLE
        ):
            self._enter(LeaderState.IDLE, now, "object released; waiting for /mission/start")
        elif now - self._release_open_time > self._c.release_open_settle + self._c.handshake_timeout:
            self._fault("Follower did not finish RELEASE", now)

    def _fault(self, reason: str, now: float) -> None:
        if self._state == LeaderState.FAULT:
            return
        self._actions.clear_waits()
        self._actions.emit(ActionKind.APPROACH_ENABLE, False, now, wait=False)
        self._actions.emit(ActionKind.GUARD_ENABLE, False, now, wait=False)
        self._send(FollowerCommand.ABORT)
        self._enter(LeaderState.FAULT, now, reason)

    def _enter(self, state: LeaderState, now: float, detail: str) -> None:
        if state == LeaderState.LEADER_SEARCH:
            self._search.reset()
        self._state = state
        self._state_since = now
        self._state_changed = True
        self._detail = detail

    def _consume_changed(self) -> bool:
        changed, self._state_changed = self._state_changed, False
        return changed

    def _send(self, command: FollowerCommand) -> None:
        self._seq += 1
        self._pending = CommandMessage(self._session, self._seq, command)
        self._pending_last_sent = None

    def _acked(self) -> bool:
        follower = self._follower
        return (
            self._pending is not None
            and follower is not None
            and follower.session == self._session
            and follower.ack_seq >= self._pending.seq
        )

    def _acked_in(self, state: FollowerState) -> bool:
        return self._acked() and self._follower is not None and self._follower.state == state

    def _follower_fresh(self, now: float, timeout: float) -> bool:
        return self._follower_time is not None and now - self._follower_time <= timeout

    def _follower_message(self, now: float) -> Optional[CommandMessage]:
        if self._pending is None or self._acked():
            return None
        if (
            self._pending_last_sent is not None
            and now - self._pending_last_sent < self._c.command_resend_period
        ):
            return None
        self._pending_last_sent = now
        return self._pending


def make_session_id(seed: float) -> str:
    """Short run identifier so a restarted Leader never reuses old sequence numbers."""
    return f"L{int(seed * 1000) & 0xFFFFFFFF:08x}"


__all__ = [
    "COOPERATION_STATE",
    "LeaderMission",
    "LeaderMissionConfig",
    "LeaderOutputs",
    "LeaderState",
    "make_session_id",
]
