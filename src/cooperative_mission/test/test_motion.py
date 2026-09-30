import math

import pytest

from cooperative_mission.motion import (
    GripperParameters,
    ManeuverExecutor,
    ManeuverParameters,
    PlanarCommand,
    Pose2D,
    SearchController,
    SearchParameters,
    TagTracker,
    gripper_close,
    gripper_lift,
    gripper_open_for_approach,
    leader_to_follower,
    parse_segments,
    transport_command,
    trapezoid_speed,
)


def test_trapezoid_is_zero_outside_window() -> None:
    assert trapezoid_speed(-0.01, 1.0, 0.05, 0.2) == 0.0
    assert trapezoid_speed(1.0, 1.0, 0.05, 0.2) == 0.0
    assert trapezoid_speed(1.5, 1.0, 0.05, 0.2) == 0.0


def test_trapezoid_shape_and_distance() -> None:
    dt = 0.001
    samples = [trapezoid_speed(i * dt, 1.0, 0.05, 0.2) for i in range(1100)]
    assert max(samples) == pytest.approx(0.05)
    assert trapezoid_speed(0.125, 1.0, 0.05, 0.2) == pytest.approx(0.025)
    assert trapezoid_speed(0.5, 1.0, 0.05, 0.2) == pytest.approx(0.05)
    assert trapezoid_speed(0.875, 1.0, 0.05, 0.2) == pytest.approx(0.025)
    # 0.25 s ramps + 0.5 s cruise -> 0.0375 m
    assert sum(samples) * dt == pytest.approx(0.0375, abs=5e-4)
    # acceleration never exceeds the configured value
    assert max(abs(b - a) / dt for a, b in zip(samples, samples[1:-200])) <= 0.2 + 1e-6


def test_trapezoid_becomes_triangle_when_window_is_short() -> None:
    assert trapezoid_speed(0.25, 0.5, 1.0, 0.2) == pytest.approx(0.05)
    assert max(trapezoid_speed(i * 0.01, 0.5, 1.0, 0.2) for i in range(60)) <= 0.05 + 1e-9


def test_transport_direction_and_follower_conversion() -> None:
    forward = transport_command(0.5, 1.0, 0.05, 0.2, 1.0)
    backward = transport_command(0.5, 1.0, 0.05, 0.2, -1.0)
    assert forward == PlanarCommand(0.05, 0.0)
    assert backward == PlanarCommand(-0.05, 0.0)
    assert leader_to_follower(forward, True) == PlanarCommand(-0.05, 0.0)
    assert leader_to_follower(forward, False) == forward
    assert leader_to_follower(PlanarCommand(0.0, 0.1), True).angular_z == 0.1


def test_tag_tracker_acquire_and_loss() -> None:
    tracker = TagTracker(target_tag_id=0, acquire_time=0.3)
    tracker.on_tag_id(0, 0.0)
    tracker.on_detected(True, 0.0)
    assert not tracker.acquired(0.2)
    assert tracker.acquired(0.31)
    tracker.on_detected(False, 1.0)
    assert not tracker.acquired(1.0)
    assert tracker.lost_for(2.5) == pytest.approx(1.5)
    tracker.on_tag_id(3, 3.0)
    tracker.on_detected(True, 3.0)
    assert not tracker.acquired(5.0)  # wrong id


def test_tag_tracker_any_id() -> None:
    tracker = TagTracker(target_tag_id=-1, acquire_time=0.0)
    tracker.on_detected(True, 0.0)
    assert not tracker.acquired(0.1)  # tag id not yet known
    tracker.on_tag_id(2, 0.1)
    assert tracker.acquired(0.1)
    assert TagTracker(-1, 0.1).lost_for(1.0) == math.inf


def test_search_steps_dwells_and_exhausts() -> None:
    params = SearchParameters(0.5, -1.0, math.radians(20.0), 0.5, math.radians(60.0))
    search = SearchController(params)
    t, rotating, commands = 0.0, 0.0, []
    while t < 10.0 and not search.exhausted():
        command = search.update(t)
        commands.append(command)
        rotating += abs(command.angular_z) * 0.02
        t += 0.02
    assert search.exhausted()
    assert all(c.angular_z <= 0.0 for c in commands)
    assert any(c.is_zero for c in commands)  # dwell periods exist
    assert rotating == pytest.approx(math.radians(60.0), abs=0.03)
    assert search.update(t).is_zero


def test_parse_segments() -> None:
    segments = parse_segments(["turn:90", " drive:0.5 ", "wait:1", ""])
    assert [s.kind for s in segments] == ["turn", "drive", "wait"]
    assert segments[0].value == pytest.approx(math.pi / 2)
    for bad in (["spin:1"], ["turn"], ["turn:x"], ["wait:-1"], ["turn:270"], ["drive:nan"]):
        with pytest.raises(ValueError):
            parse_segments(bad)


def _simulate(executor: ManeuverExecutor, pose: Pose2D, seconds: float = 60.0) -> Pose2D:
    t, dt = 0.0, 0.02
    while t < seconds and not executor.done and executor.failure is None:
        command = executor.update(t, pose)
        yaw = pose.yaw + command.angular_z * dt
        pose = Pose2D(
            pose.x + command.linear_x * math.cos(yaw) * dt,
            pose.y + command.linear_x * math.sin(yaw) * dt,
            math.atan2(math.sin(yaw), math.cos(yaw)),
        )
        t += dt
    return pose


def test_maneuver_turn_then_drive_with_odometry() -> None:
    executor = ManeuverExecutor(
        parse_segments(["turn:90", "drive:0.5", "turn:-180"]), ManeuverParameters()
    )
    final = _simulate(executor, Pose2D(0.0, 0.0, 0.0))
    assert executor.done and executor.failure is None
    assert final.x == pytest.approx(0.0, abs=0.03)
    assert final.y == pytest.approx(0.5, abs=0.02)
    heading_error = math.atan2(math.sin(final.yaw + math.pi / 2), math.cos(final.yaw + math.pi / 2))
    assert abs(heading_error) < math.radians(3)  # ends facing -y


def test_maneuver_times_out_without_odometry_progress() -> None:
    executor = ManeuverExecutor(parse_segments(["drive:0.2"]), ManeuverParameters())
    pose = Pose2D(0.0, 0.0, 0.0)
    t = 0.0
    while t < 30.0 and executor.failure is None:
        executor.update(t, pose)  # wheels slip: pose never changes
        t += 0.1
    assert executor.failure is not None and "timed out" in executor.failure


def test_gripper_commands_keep_jaw_torque() -> None:
    p = GripperParameters(open_raw=950, close_raw=350, lift_raw=300, lower_raw=600)
    p.validate()
    assert gripper_open_for_approach(p) == (-1.0, 950.0, -1.0, 1.0)
    assert gripper_close(p) == (-1.0, 350.0, -1.0, 1.0)
    assert gripper_lift(p) == (300.0, -1.0, 1.0, -1.0)
    with_arm = GripperParameters(950, 350, 300, 600, approach_rx64_raw=600)
    assert gripper_open_for_approach(with_arm) == (600.0, 950.0, 1.0, 1.0)
    with pytest.raises(ValueError):
        GripperParameters(950, 350, 700, 600).validate()
    with pytest.raises(ValueError):
        GripperParameters(0, 350, 300, 600).validate()
