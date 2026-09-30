"""End-to-end scenario tests: both state machines + simulated robots and link."""

import dataclasses

import pytest

from cooperative_mission.actions import ActionKind
from cooperative_mission.follower_logic import FollowerMissionConfig
from cooperative_mission.leader_logic import LeaderMissionConfig, LeaderState
from cooperative_mission.protocol import FollowerState
from sim_world import DT, World


def start(world: World) -> None:
    world.settle(0.5)  # exchange heartbeats/status first
    ok, message = world.leader.start(world.t)
    assert ok, message


def run_to_done(world: World) -> None:
    start(world)
    assert world.run_until(lambda w: w.leader.state in (LeaderState.DONE, LeaderState.FAULT), 200.0)
    assert world.leader.state == LeaderState.DONE, world.leader.detail


def visited(world: World):
    order = []
    for _, state in world.leader_states:
        if not order or order[-1] != state:
            order.append(state)
    return order


def test_full_scenario_runs_in_order() -> None:
    world = World()
    run_to_done(world)
    assert visited(world) == [
        LeaderState.IDLE,
        LeaderState.LEADER_SEARCH,
        LeaderState.LEADER_APPROACH,
        LeaderState.LEADER_GRASP,
        LeaderState.FOLLOWER_APPROACH,
        LeaderState.LIFT_PREPARE,
        LeaderState.LIFT,
        LeaderState.TRANSPORT_PREPARE,
        LeaderState.TRANSPORT,
        LeaderState.TRANSPORT_FINISH,
        LeaderState.DONE,
    ]
    assert world.follower.state == FollowerState.HOLD
    # Leader searched (rotated) before it could see the tag.
    assert world.L.yaw_swept >= world.L.tag_visible_after_rad


def test_leader_grasps_before_follower_moves() -> None:
    world = World()
    run_to_done(world)
    leader_close = next(t for t, v in world.L.gripper_log if v == (-1.0, 450.0, -1.0, 1.0))
    follower_open = next(t for t, v in world.F.gripper_log if v[1] == 950.0)
    follower_close = next(t for t, v in world.F.gripper_log if v == (-1.0, 350.0, -1.0, 1.0))
    assert leader_close < follower_open < follower_close


def test_lift_is_simultaneous_and_follower_never_late() -> None:
    world = World(latency=0.004)
    run_to_done(world)
    assert len(world.L.lift_times) == 1 and len(world.F.lift_times) == 1
    offset = world.L.lift_times[0] - world.F.lift_times[0]
    # Leader lifts on the Follower acknowledgement: one link hop, never before.
    assert 0.0 <= offset <= 2 * 0.004 + 1e-9
    # Both lifts happen only after both grippers are closed.
    follower_close = next(t for t, v in world.F.gripper_log if v[1] == 350.0)
    assert world.F.lift_times[0] > follower_close


def test_transport_moves_both_robots_same_direction_for_one_second() -> None:
    world = World()
    x_leader0 = world.L.world_x
    run_to_done(world)
    moving = [(t, vl, vf) for t, vl, vf in world.transport_log if abs(vl) > 1e-9 or abs(vf) > 1e-9]
    assert moving, "no transport motion"
    duration_leader = sum(DT for _, vl, _ in moving if abs(vl) > 1e-9)
    duration_follower = sum(DT for _, _, vf in moving if abs(vf) > 1e-9)
    assert duration_leader == pytest.approx(1.0, abs=2 * DT)
    assert duration_follower == pytest.approx(1.0, abs=2 * DT)
    # Same world direction (Leader forward = +x; Follower reverses in its frame).
    assert all(vl >= 0.0 and vf >= 0.0 for _, vl, vf in moving)
    # The two velocities never differ by more than one acceleration step.
    assert max(abs(vl - vf) for _, vl, vf in moving) <= 0.2 * 2 * DT + 1e-9
    moved_leader = world.L.world_x - x_leader0
    assert moved_leader > 0.03


def test_backward_transport_direction() -> None:
    world = World(leader_config=LeaderMissionConfig(transport_direction="backward"))
    run_to_done(world)
    moving = [(vl, vf) for _, vl, vf in world.transport_log if abs(vl) > 1e-9]
    assert moving and all(vl < 0.0 for vl, _ in moving)
    assert all(vf <= 0.0 for _, _, vf in world.transport_log)


def test_follower_heartbeat_loss_during_transport_stops_leader() -> None:
    world = World()
    start(world)
    assert world.run_until(lambda w: w.leader.state == LeaderState.TRANSPORT, 200.0)
    world.step()
    world.link.cut_follower_to_leader = True
    assert world.run_until(lambda w: w.leader.state == LeaderState.FAULT, 1.0)
    fault_time = world.t
    world.settle(0.2)
    assert world.L.world_velocity == 0.0
    assert "Follower status timeout" in world.leader.detail
    assert fault_time - world.transport_log[0][0] < 0.35 + 0.1
    # The Follower receives ABORT and stops too.
    world.settle(0.3)
    assert world.follower.state == FollowerState.FAULT
    assert world.F.world_velocity == 0.0
    assert not world.F.guard_enabled


def test_leader_link_loss_stops_follower() -> None:
    world = World()
    start(world)
    assert world.run_until(lambda w: w.leader.state == LeaderState.TRANSPORT, 200.0)
    world.link.cut_leader_to_follower = True
    assert world.run_until(lambda w: w.follower.state == FollowerState.FAULT, 1.5)
    assert "Leader heartbeat" in world.follower.detail
    world.settle(0.1)
    assert world.F.world_velocity == 0.0


def test_follower_not_idle_rejects_start() -> None:
    world = World()
    world.settle(0.5)
    world.follower._state = FollowerState.HOLD  # stale state from a previous run
    world.settle(0.3)
    ok, message = world.leader.start(world.t)
    assert not ok and "expected IDLE" in message
    ok, _ = world.leader.reset(world.t)
    assert ok
    world.settle(0.5)
    assert world.follower.state == FollowerState.IDLE
    ok, message = world.leader.start(world.t)
    assert ok, message


def test_follower_absent_rejects_start() -> None:
    world = World()
    world.link.cut_follower_to_leader = True
    world.settle(1.5)
    ok, message = world.leader.start(world.t)
    assert not ok and "Follower" in message


def test_follower_not_ready_rejects_start() -> None:
    world = World()
    world.follower.set_ready(False, "no Follower Dynamixel node subscribed")
    world.settle(0.5)
    ok, message = world.leader.start(world.t)
    assert not ok and "Dynamixel" in message


def test_service_failure_faults_and_stops() -> None:
    world = World()
    world.F.fail_kind = ActionKind.SELECTOR_MODE
    start(world)
    assert world.run_until(lambda w: w.leader.state == LeaderState.FAULT, 200.0)
    assert world.follower.state == FollowerState.FAULT
    assert "SELECTOR_MODE" in world.follower.detail
    assert world.L.lift_times == [] and world.F.lift_times == []


def test_missing_service_response_times_out_to_fault() -> None:
    world = World()
    world.L.drop_confirmations = True
    start(world)
    assert world.run_until(lambda w: w.leader.state == LeaderState.FAULT, 5.0)
    assert "no confirmation" in world.leader.detail


def test_leader_tag_never_found_faults_after_full_sweep() -> None:
    world = World()
    world.L.tag_visible_after_rad = 100.0
    start(world)
    assert world.run_until(lambda w: w.leader.state == LeaderState.FAULT, 200.0)
    assert "not found" in world.leader.detail
    assert world.L.yaw_swept == pytest.approx(6.283, abs=0.05)
    assert world.follower.state == FollowerState.IDLE  # never commanded


def test_follower_reposition_then_search() -> None:
    config = FollowerMissionConfig(reposition_segments=("wait:0.5", "drive:0.1"))
    world = World(follower_config=config)
    run_to_done(world)
    assert world.follower.state == FollowerState.HOLD


def test_release_after_done_returns_both_to_idle_and_lowers_together() -> None:
    world = World()
    run_to_done(world)
    ok, message = world.leader.release(world.t)
    assert ok, message
    assert world.run_until(lambda w: w.leader.state in (LeaderState.IDLE, LeaderState.FAULT), 30.0)
    assert world.leader.state == LeaderState.IDLE, world.leader.detail
    world.settle(0.3)
    assert world.follower.state == FollowerState.IDLE
    lower_l = [t for t, v in world.L.gripper_log if v[0] == 600.0]
    lower_f = [t for t, v in world.F.gripper_log if v[0] == 600.0]
    assert lower_l and lower_f and 0.0 <= lower_l[0] - lower_f[0] <= 0.01
    # Mission can run again after release, even though both robots are still
    # latched ALIGNED at their grasp poses from the first run.
    ok, message = world.leader.start(world.t)
    assert ok, message
    assert world.run_until(lambda w: w.leader.state in (LeaderState.DONE, LeaderState.FAULT), 200.0)
    assert world.leader.state == LeaderState.DONE, world.leader.detail
    assert len(world.L.lift_times) == 2 and len(world.F.lift_times) == 2


def test_abort_during_follower_approach() -> None:
    world = World()
    start(world)
    assert world.run_until(lambda w: w.follower.state == FollowerState.APPROACH, 200.0)
    world.leader.abort(world.t)
    world.settle(0.3)
    assert world.leader.state == LeaderState.FAULT
    assert world.follower.state == FollowerState.FAULT
    assert not world.F.guard_enabled and not world.F.approach_enabled
    assert world.F.selector == "STOP"


def test_command_retransmission_is_idempotent() -> None:
    world = World(latency=0.12)  # round trip 0.24 s > 0.2 s resend period
    run_to_done(world)
    assert len(world.received_seqs) > len(set(world.received_seqs))  # resends happened
    assert len(world.F.lift_times) == 1
    assert len([1 for _, v in world.F.gripper_log if v[1] == 350.0]) == 1


def test_config_validation_rejects_unsafe_transport() -> None:
    with pytest.raises(ValueError):
        LeaderMissionConfig(transport_speed=0.5).validate()
    with pytest.raises(ValueError):
        LeaderMissionConfig(transport_direction="left").validate()
    with pytest.raises(ValueError):
        dataclasses.replace(LeaderMissionConfig(), transport_duration=0.0).validate()
    with pytest.raises(ValueError):
        FollowerMissionConfig(reposition_segments=("fly:1",)).validate()


def test_stale_aligned_sample_before_phase_is_not_trusted() -> None:
    from cooperative_mission.leader_logic import LeaderMission

    mission = LeaderMission(LeaderMissionConfig(require_follower=False), "Lx")
    mission.on_alignment_state("ALIGNED", 0.0)  # old sample, then perception went silent
    mission.start(1.0)
    mission.update(1.0)
    for action_id in range(1, 4):
        mission.on_action_result(action_id, True, "", 1.01)
    mission.on_tag_id(0, 1.02)
    mission.on_tag_detected(True, 1.02)
    for step in range(60):
        now = 1.02 + 0.02 * step
        mission.on_tag_detected(True, now)
        mission.update(now)
    assert mission.state == LeaderState.LEADER_APPROACH  # no fresh ALIGNED sample
    mission.on_alignment_state("ALIGNED", 2.3)
    mission.update(2.3)
    mission.on_alignment_state("ALIGNED", 2.55)
    mission.on_tag_detected(True, 2.55)
    mission.update(2.55)
    assert mission.state == LeaderState.LEADER_GRASP
