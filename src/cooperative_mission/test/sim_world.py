"""Deterministic two-robot simulation of the mission logic (no ROS required).

It mirrors what the two nodes do each 50 Hz tick: execute emitted actions,
publish velocity/state topics, relay messages over a latency-delayed "DDS"
link, and feed perception/odometry back. Robots move on a 1-D world axis:
the Leader faces +x and the Follower faces -x (they hold opposite faces).
"""

from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from cooperative_mission.actions import ActionKind
from cooperative_mission.follower_logic import FollowerMission, FollowerMissionConfig
from cooperative_mission.leader_logic import LeaderMission, LeaderMissionConfig, LeaderState
from cooperative_mission.motion import ZERO, PlanarCommand
from cooperative_mission.protocol import (
    decode_command,
    decode_status,
    encode_command,
    encode_status,
)

DT = 0.02


@dataclass
class RobotSim:
    name: str
    approach_enabled: bool = False
    guard_enabled: bool = False
    selector: str = "STOP"
    yaw_swept: float = 0.0
    tag_visible_after_rad: float = 0.0
    approach_time_to_align: float = 2.0
    approach_active_since: Optional[float] = None
    aligned: bool = False
    world_x: float = 0.0
    world_velocity: float = 0.0
    lift_times: List[float] = field(default_factory=list)
    gripper_log: List[Tuple[float, Tuple[float, ...]]] = field(default_factory=list)
    fail_kind: Optional[ActionKind] = None
    drop_confirmations: bool = False


class Link:
    """Priority queue of (deliver_time, order, callback)."""

    def __init__(self, latency: float) -> None:
        self.latency = latency
        self._queue: list = []
        self._order = itertools.count()
        self.cut_leader_to_follower = False
        self.cut_follower_to_leader = False

    def send(self, now: float, callback: Callable[[float], None]) -> None:
        heapq.heappush(self._queue, (now + self.latency, next(self._order), callback))

    def deliver(self, now: float) -> None:
        while self._queue and self._queue[0][0] <= now + 1e-12:
            deliver_at, _, callback = heapq.heappop(self._queue)
            callback(deliver_at)


class World:
    def __init__(
        self,
        leader_config: Optional[LeaderMissionConfig] = None,
        follower_config: Optional[FollowerMissionConfig] = None,
        latency: float = 0.004,
    ) -> None:
        self.t = 0.0
        self.leader = LeaderMission(leader_config or LeaderMissionConfig(), "Ltest")
        self.follower = FollowerMission(follower_config or FollowerMissionConfig())
        self.follower.set_ready(True)
        self.L = RobotSim("leader", tag_visible_after_rad=0.6)
        self.F = RobotSim("follower", tag_visible_after_rad=0.0, world_x=0.8)
        self.link = Link(latency)
        self.pending_results: List[Tuple[float, Callable[[], None]]] = []
        self.leader_states: List[Tuple[float, LeaderState]] = []
        self.transport_log: List[Tuple[float, float, float]] = []  # t, vL_world, vF_world
        self._last_heartbeat = -1.0
        self._last_status = -1.0
        self.leader_state_log: List[str] = []
        self.received_seqs: List[int] = []

    # ------------------------------------------------------------ execution
    def _execute(self, robot: RobotSim, logic, action, now: float) -> None:
        success = True
        if robot.fail_kind == action.kind and action.value not in (False, "STOP"):
            success = False
        if action.kind == ActionKind.APPROACH_ENABLE:
            robot.approach_enabled = bool(action.value) if success else robot.approach_enabled
            robot.approach_active_since = None
        elif action.kind == ActionKind.GUARD_ENABLE:
            robot.guard_enabled = bool(action.value) if success else robot.guard_enabled
        elif action.kind == ActionKind.SELECTOR_MODE:
            robot.selector = str(action.value) if success else robot.selector
        elif action.kind == ActionKind.GRIPPER:
            robot.gripper_log.append((now, tuple(action.value)))
            if action.value[0] == 300.0 and action.value[1] < 0:  # RX-64 lift_raw
                robot.lift_times.append(now)
        if robot.drop_confirmations:
            return
        # Service responses arrive one tick later; topic actions immediately.
        delay = 0.0 if action.kind == ActionKind.GRIPPER else DT
        self.pending_results.append(
            (now + delay, lambda: logic.on_action_result(action.action_id, success, "sim", self.t))
        )

    def _perception(self, robot: RobotSim, logic, now: float, uses_selector: bool) -> None:
        visible = robot.yaw_swept >= robot.tag_visible_after_rad
        logic.on_tag_id(0 if visible else -1, now)
        logic.on_tag_detected(visible, now)
        controlling = robot.approach_enabled and robot.guard_enabled and (
            not uses_selector or robot.selector == "APPROACH"
        )
        if controlling and visible:
            if robot.approach_active_since is None:
                robot.approach_active_since = now
            if now - robot.approach_active_since >= robot.approach_time_to_align:
                robot.aligned = True
        state = "ALIGNED" if robot.aligned and visible else ("APPROACH" if visible else "TAG_LOST")
        logic.on_alignment_state(state, now)
        if robot is self.L:
            cmd = (0.0, 0.0) if robot.aligned or not visible else (0.03, 0.01)
            if robot.approach_enabled:
                logic.on_approach_command(cmd[0], cmd[1], now)

    # ----------------------------------------------------------------- step
    def step(self) -> None:
        now = self.t
        self.link.deliver(now)
        due = [r for r in self.pending_results if r[0] <= now + 1e-12]
        self.pending_results = [r for r in self.pending_results if r[0] > now + 1e-12]
        for _, fn in due:
            fn()

        self._perception(self.L, self.leader, now, uses_selector=False)
        self._perception(self.F, self.follower, now, uses_selector=True)
        self.follower.on_odometry(self.F.world_x, 0.0, 3.14159, now)

        # ---- Leader tick
        out = self.leader.update(now)
        for action in out.actions:
            self._execute(self.L, self.leader, action, now)
        v_leader = out.drive if self.L.guard_enabled else ZERO
        self.L.yaw_swept += abs(v_leader.angular_z) * DT
        self.L.world_velocity = v_leader.linear_x
        target = out.target_velocity
        if not self.link.cut_leader_to_follower:
            self.link.send(now, lambda at, v=target: self.follower.on_target_velocity(
                v.linear_x, v.angular_z, at))
            if out.follower_message is not None:
                text = encode_command(out.follower_message)
                self.link.send(now, lambda at, s=text: self._follower_receive(s, at))
            if now - self._last_heartbeat >= 0.2 or out.state_changed:
                self._last_heartbeat = now
                state = out.state.value
                self.link.send(now, lambda at, s=state: self.follower.on_leader_state(s, at))
        self.leader_states.append((now, out.state))

        # ---- Follower tick
        fout = self.follower.update(now)
        for action in fout.actions:
            self._execute(self.F, self.follower, action, now)
        f_cmd = fout.cooperation_command
        moving = self.F.guard_enabled and self.F.selector == "COOPERATION"
        v_follower = f_cmd if moving else ZERO
        if self.F.selector == "APPROACH" and self.F.guard_enabled:
            v_follower = PlanarCommand(0.02, 0.0) if not self.F.aligned else ZERO
        self.F.yaw_swept += abs(v_follower.angular_z) * DT
        self.F.world_velocity = -v_follower.linear_x  # faces -x
        if now - self._last_status >= 0.1 or fout.status_changed:
            self._last_status = now
            self._send_status()

        self.L.world_x += self.L.world_velocity * DT
        self.F.world_x += self.F.world_velocity * DT
        if out.state == LeaderState.TRANSPORT or self.leader.state == LeaderState.TRANSPORT_FINISH:
            self.transport_log.append((now, self.L.world_velocity, self.F.world_velocity))
        self.t = round(self.t + DT, 10)

    def _follower_receive(self, text: str, at: float) -> None:
        command = decode_command(text)
        assert command is not None
        self.received_seqs.append(command.seq)
        self.follower.on_command(command, at)
        for action in self.follower.take_actions():
            self._execute(self.F, self.follower, action, at)
        self._send_status(at)

    def _send_status(self, at: Optional[float] = None) -> None:
        if self.link.cut_follower_to_leader:
            return
        text = encode_status(self.follower.status())
        now = self.t if at is None else at

        def deliver(when: float, s: str = text) -> None:
            status = decode_status(s)
            assert status is not None
            self.leader.on_follower_status(status, when)
            if self.leader.state in (LeaderState.LIFT, LeaderState.RELEASE):
                out = self.leader.update(when)
                for action in out.actions:
                    self._execute(self.L, self.leader, action, when)
                if out.follower_message is not None:
                    msg = encode_command(out.follower_message)
                    self.link.send(when, lambda a, m=msg: self._follower_receive(m, a))

        self.link.send(now, deliver)

    def run_until(self, predicate: Callable[["World"], bool], timeout: float = 120.0) -> bool:
        end = self.t + timeout
        while self.t < end:
            if predicate(self):
                return True
            self.step()
        return predicate(self)

    def settle(self, seconds: float = 0.5) -> None:
        self.run_until(lambda w: False, timeout=seconds)
