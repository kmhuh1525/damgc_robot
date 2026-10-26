"""Focused tests for receipt timing, atomic commands, and blind-final safety."""

from math import radians
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from std_msgs.msg import Bool

from follower_supply_perception.approach_logic import ApproachState, TagObservation
import follower_supply_perception.apriltag_approach_node as node_module
from follower_supply_perception.apriltag_approach_node import AprilTagApproachNode
from follower_supply_perception.base_alignment_logic import AlignmentDecision, ControlMode


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_source_stamp_identifies_new_samples_but_receipt_time_drives_age() -> None:
    harness = SimpleNamespace(
        _last_processed_observation_stamps={}, _last_observation_receipts={}
    )
    sample = TagObservation(0, 0.0, 0.0, 1.0, (0.0, 0.0, 0.0, 1.0), 100)

    assert AprilTagApproachNode._accept_observation_stamp(harness, sample, 50.0)
    assert harness._last_observation_receipts[0] == 50.0
    assert not AprilTagApproachNode._accept_observation_stamp(harness, sample, 60.0)
    assert harness._last_observation_receipts[0] == 50.0

    newer = TagObservation(0, 0.0, 0.0, 1.0, sample.quaternion, 101)
    assert AprilTagApproachNode._accept_observation_stamp(harness, newer, 60.0)
    assert harness._last_observation_receipts[0] == 60.0


def test_atomic_command_keeps_pose_mode_and_state_in_one_generation() -> None:
    publisher = RecordingPublisher()
    harness = SimpleNamespace(
        _base_frame="base_link",
        _command_pub=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
    )
    first_pose = PoseStamped()
    first_pose.header.frame_id = "base_link"
    first_pose.header.stamp = Time(seconds=10.0).to_msg()
    first_pose.pose.position.x = 0.3
    first = AlignmentDecision(
        ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
    )
    AprilTagApproachNode._publish_atomic_command(harness, first_pose, first)

    second = AlignmentDecision(ApproachState.TAG_LOST, ControlMode.TAG_LOST)
    AprilTagApproachNode._publish_atomic_command(harness, None, second)

    assert publisher.messages[0].header == first_pose.header
    assert publisher.messages[0].target_pose.position.x == pytest.approx(0.3)
    assert publisher.messages[0].control_mode == "FINAL_APPROACH"
    assert publisher.messages[0].alignment_state == "FINAL_APPROACH"
    assert publisher.messages[1].target_pose.position.x == 0.0
    assert publisher.messages[1].control_mode == "TAG_LOST"
    assert publisher.messages[1].alignment_state == "TAG_LOST"


def test_odom_requires_fresh_source_and_local_receipt_times() -> None:
    harness = SimpleNamespace(
        _last_odom=(1.0, 2.0, 0.1, 10.0, 100.0),
        _blind_odom_timeout=0.25,
    )
    assert AprilTagApproachNode._fresh_odom(harness, 10.2, 100.2) == (
        1.0,
        2.0,
        0.1,
    )
    assert AprilTagApproachNode._fresh_odom(harness, 10.3, 100.2) is None
    assert AprilTagApproachNode._fresh_odom(harness, 10.2, 100.3) is None
    harness._last_odom = None
    assert AprilTagApproachNode._fresh_odom(harness, 10.0, 100.0) is None


def blind_cycle_harness(current_odom) -> SimpleNamespace:
    commands = []
    losses = []
    harness = SimpleNamespace(
        _blind_active=True,
        _blind_completed=False,
        _blind_start_receipt=10.0,
        _blind_max_duration=5.0,
        _blind_start_odom=(0.0, 0.0, 0.0),
        _blind_previous_odom=(0.0, 0.0, 0.0),
        _blind_reverse_tolerance=0.01,
        _blind_max_odom_step=0.05,
        _blind_max_lateral_deviation=0.03,
        _blind_max_yaw_deviation_deg=12.0,
        _blind_max_distance=0.10,
        _blind_planned_distance=0.08,
        _last_odom_progress=0.0,
        _last_valid_tag_x=0.33,
        _last_valid_receipt=9.9,
        _last_valid_yaw_error=0.0,
        _last_valid_cross_track=0.0,
        _fresh_odom=lambda *_args: current_odom,
        _publish_blind_command=lambda state, mode: commands.append((state, mode)),
        _publish_base_lost=lambda *_args: losses.append(True),
    )
    harness._clear_blind_plan = lambda: AprilTagApproachNode._clear_blind_plan(
        harness
    )
    harness._clear_final_sample = lambda: AprilTagApproachNode._clear_final_sample(
        harness
    )
    harness._abort_blind = lambda *args: AprilTagApproachNode._abort_blind(
        harness, *args
    )
    harness.commands = commands
    harness.losses = losses
    return harness


@pytest.mark.parametrize(
    "current_odom",
    [
        None,
        (0.06, 0.0, 0.0),  # odometry jump
        (0.0, 0.04, 0.0),  # lateral deviation
        (0.0, 0.0, radians(13.0)),  # yaw deviation
        (-0.02, 0.0, 0.0),  # reverse motion
    ],
)
def test_blind_safety_violation_aborts_to_loss(current_odom) -> None:
    harness = blind_cycle_harness(current_odom)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.losses == [True]
    assert not harness._blind_active
    assert harness.commands == []


def test_blind_timeout_aborts() -> None:
    harness = blind_cycle_harness((0.0, 0.0, 0.0))
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 15.1)
    assert harness.losses == [True]


def test_blind_total_distance_limit_aborts_independently_of_step_limit() -> None:
    harness = blind_cycle_harness((0.11, 0.0, 0.0))
    harness._blind_previous_odom = (0.08, 0.0, 0.0)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.losses == [True]


def test_blind_progress_continues_then_completes_aligned() -> None:
    harness = blind_cycle_harness((0.02, 0.0, 0.0))
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.commands[-1] == (
        ApproachState.FINAL_APPROACH,
        ControlMode.BLIND_FINAL_APPROACH,
    )
    assert harness._last_odom_progress == pytest.approx(0.02)

    harness._blind_previous_odom = (0.04, 0.0, 0.0)
    harness._fresh_odom = lambda *_args: (0.08, 0.0, 0.0)
    AprilTagApproachNode._publish_blind_cycle(harness, 20.1, 10.2)
    assert harness.commands[-1] == (ApproachState.ALIGNED, ControlMode.ALIGNED)
    assert harness._blind_completed
    assert not harness._blind_active


def test_odom_yaw_projection_uses_start_heading() -> None:
    harness = blind_cycle_harness((0.0, 0.02, radians(90.0)))
    harness._blind_start_odom = (0.0, 0.0, radians(90.0))
    harness._blind_previous_odom = harness._blind_start_odom
    harness._blind_planned_distance = 0.02
    AprilTagApproachNode._publish_blind_cycle(harness, 20.0, 10.1)
    assert harness.commands[-1] == (ApproachState.ALIGNED, ControlMode.ALIGNED)


def test_fresh_visual_sample_cancels_blind_and_resumes_visual(monkeypatch) -> None:
    sample = TagObservation(
        0, 0.0, 0.0, 0.3, (0.0, 0.0, 0.0, 1.0), 19_500_000_000
    )
    published = []
    cleared = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=True,
        _selection_mode="priority",
        _active_tag_id=0,
        _last_logged_base_state=ApproachState.APPROACH,
        _tag_receipt_timeout=0.35,
        _blind_last_tag_max_age=0.25,
        _last_valid_receipt=19.0,
        _last_processed_observation_stamps={},
        _last_observation_receipts={},
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda *_args: sample,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
        _translation_filter=SimpleNamespace(add=lambda *_args: "measurement"),
        _normal_filter=SimpleNamespace(reset=lambda: None),
        _state_machine=SimpleNamespace(
            update=lambda *_args: ApproachState.APPROACH, reset=lambda: None
        ),
        _base_state_machine=SimpleNamespace(reset=lambda: None),
        _publish_valid=lambda *args: published.append(args),
    )

    def clear_blind() -> None:
        harness._blind_active = False
        cleared.append(True)

    harness._clear_blind_plan = clear_blind
    harness._accept_observation_stamp = lambda observation, receipt: (
        AprilTagApproachNode._accept_observation_stamp(harness, observation, receipt)
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 100.0)

    AprilTagApproachNode._on_timer(harness)

    assert cleared == [True]
    assert len(published) == 1
    assert published[0][-1] is True


def final_grace_harness(*, blind_enabled: bool = False) -> SimpleNamespace:
    atomic_commands = []
    lost_cycles = []
    logs = []
    harness = SimpleNamespace(
        _last_fresh_final_observation_receipt=10.0,
        _final_approach_grace_eligible=True,
        _final_approach_grace_active=False,
        _final_approach_tag_loss_grace_sec=0.30,
        _blind_final_approach_enabled=blind_enabled,
        _detected_pub=RecordingPublisher(),
        _tag_id_pub=RecordingPublisher(),
        _control_mode_pub=RecordingPublisher(),
        _base_state_pub=RecordingPublisher(),
        _publish_atomic_command=lambda pose, decision: atomic_commands.append(
            (pose, decision)
        ),
        _publish_blind_diagnostics=lambda active: None,
        _log_base_state_change=lambda state: None,
        _publish_lost=lambda ros_now, receipt_now, *, allow_blind=True: (
            lost_cycles.append((ros_now, receipt_now, allow_blind))
        ),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
        get_logger=lambda: SimpleNamespace(info=logs.append),
    )
    harness._clear_final_approach_grace = lambda: (
        AprilTagApproachNode._clear_final_approach_grace(harness)
    )
    harness._expire_final_approach_grace = lambda ros_now, receipt_now: (
        AprilTagApproachNode._expire_final_approach_grace(
            harness, ros_now, receipt_now
        )
    )
    harness.atomic_commands = atomic_commands
    harness.lost_cycles = lost_cycles
    harness.logs = logs
    return harness


def test_final_approach_short_loss_holds_state_mode_and_zero_target() -> None:
    harness = final_grace_harness()

    assert AprilTagApproachNode._publish_final_approach_grace(harness, 10.29)

    assert harness._detected_pub.messages[-1].data is False
    assert harness._tag_id_pub.messages[-1].data == -1
    pose, decision = harness.atomic_commands[-1]
    assert pose is None
    assert decision == AlignmentDecision(
        ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
    )
    assert decision.control_x == decision.control_y == 0.0
    assert harness._control_mode_pub.messages[-1].data == "FINAL_APPROACH"
    assert harness._base_state_pub.messages[-1].data == "FINAL_APPROACH"
    assert not harness.lost_cycles


@pytest.mark.parametrize("blind_enabled", [False, True])
def test_final_approach_grace_expiry_controls_blind_permission(
    blind_enabled: bool,
) -> None:
    harness = final_grace_harness(blind_enabled=blind_enabled)
    AprilTagApproachNode._publish_final_approach_grace(harness, 10.1)

    assert AprilTagApproachNode._publish_final_approach_grace(harness, 10.300001)
    assert harness.lost_cycles == [
        (pytest.approx(20.0), pytest.approx(10.300001), blind_enabled)
    ]
    assert harness._last_fresh_final_observation_receipt is None
    assert not harness._final_approach_grace_eligible


def test_only_new_final_observation_resets_grace_timer() -> None:
    harness = final_grace_harness()
    final = AlignmentDecision(
        ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
    )

    AprilTagApproachNode._record_final_approach_observation(
        harness, final, 10.1, False
    )
    assert harness._last_fresh_final_observation_receipt == pytest.approx(10.0)

    harness._final_approach_grace_active = True
    AprilTagApproachNode._record_final_approach_observation(
        harness, final, 10.12, True
    )
    assert harness._last_fresh_final_observation_receipt == pytest.approx(10.12)
    assert not harness._final_approach_grace_active
    assert any("reacquired" in message for message in harness.logs)


def test_fresh_reacquisition_within_grace_resumes_visual_publish(monkeypatch) -> None:
    selected = TagObservation(
        0, 0.0, 0.0, 0.3, (0.0, 0.0, 0.0, 1.0), 10_120_000_000
    )
    published = []
    lost = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=False,
        _blind_final_approach_enabled=False,
        _final_approach_grace_active=True,
        _selection_mode="priority",
        _active_tag_id=0,
        _last_logged_base_state=ApproachState.APPROACH,
        _tag_receipt_timeout=0.35,
        _blind_last_tag_max_age=0.25,
        _last_valid_receipt=10.0,
        _last_processed_observation_stamps={0: 10_000_000_000},
        _last_observation_receipts={0: 10.0},
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda *_args: selected,
        _accept_observation_stamp=lambda observation, receipt: (
            AprilTagApproachNode._accept_observation_stamp(
                harness, observation, receipt
            )
        ),
        _expire_final_approach_grace=lambda ros_now, receipt_now: False,
        _translation_filter=SimpleNamespace(add=lambda *_args: "measurement"),
        _normal_filter=SimpleNamespace(reset=lambda: None),
        _state_machine=SimpleNamespace(
            update=lambda *_args: ApproachState.FINAL_APPROACH,
            reset=lambda: None,
        ),
        _base_state_machine=SimpleNamespace(reset=lambda: None),
        _publish_valid=lambda *args: published.append(args),
        _publish_lost=lambda *args: lost.append(args),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: 10.12)

    AprilTagApproachNode._on_timer(harness)

    assert len(published) == 1
    assert published[0][0] is selected
    assert published[0][-1] is True
    assert not lost


def duplicate_timer_harness(*, blind_enabled: bool, receipt_now: float):
    selected = TagObservation(
        0, 0.0, 0.0, 0.3, (0.0, 0.0, 0.0, 1.0), 10_000_000_000
    )
    published = []
    lost = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=False,
        _blind_final_approach_enabled=blind_enabled,
        _selection_mode="priority",
        _active_tag_id=0,
        _last_logged_base_state=ApproachState.APPROACH,
        _tag_receipt_timeout=0.35,
        _blind_last_tag_max_age=0.25,
        _last_valid_receipt=10.0,
        _last_processed_observation_stamps={0: selected.stamp_nanoseconds},
        _last_observation_receipts={0: 10.0},
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda *_args: selected,
        _accept_observation_stamp=lambda observation, receipt: (
            AprilTagApproachNode._accept_observation_stamp(
                harness, observation, receipt
            )
        ),
        _translation_filter=SimpleNamespace(add=lambda *_args: "measurement"),
        _normal_filter=SimpleNamespace(reset=lambda: None),
        _state_machine=SimpleNamespace(
            update=lambda *_args: ApproachState.FINAL_APPROACH,
            reset=lambda: None,
        ),
        _base_state_machine=SimpleNamespace(reset=lambda: None),
        _publish_valid=lambda *args: published.append(args),
        _publish_lost=lambda *args: lost.append(args),
        _publish_final_approach_grace=lambda receipt: False,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20.0)),
    )
    return harness, published, lost, receipt_now


def test_blind_age_does_not_expire_general_visual_tracking_when_disabled(
    monkeypatch,
) -> None:
    harness, published, lost, receipt_now = duplicate_timer_harness(
        blind_enabled=False, receipt_now=10.26
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)

    AprilTagApproachNode._on_timer(harness)

    assert len(published) == 1
    assert published[0][-1] is False
    assert not lost


def test_duplicate_final_sample_holds_latest_command_between_frames(
    monkeypatch,
) -> None:
    harness, published, lost, receipt_now = duplicate_timer_harness(
        blind_enabled=False, receipt_now=10.1
    )
    grace_calls = []
    harness._last_logged_base_state = ApproachState.FINAL_APPROACH
    harness._publish_final_approach_grace = lambda receipt: (
        grace_calls.append(receipt) or True
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)

    AprilTagApproachNode._on_timer(harness)

    assert not grace_calls
    assert len(published) == 1
    assert published[0][-1] is False
    assert not lost


def test_duplicate_stabilizing_sample_preserves_fresh_measurement(
    monkeypatch,
) -> None:
    harness, published, lost, receipt_now = duplicate_timer_harness(
        blind_enabled=False, receipt_now=10.1
    )
    harness._last_logged_base_state = ApproachState.STABILIZING
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)

    AprilTagApproachNode._on_timer(harness)

    assert len(published) == 1
    assert published[0][-1] is False
    assert not lost


def test_blind_age_still_triggers_existing_handoff_path_when_enabled(
    monkeypatch,
) -> None:
    harness, published, lost, receipt_now = duplicate_timer_harness(
        blind_enabled=True, receipt_now=10.26
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)

    AprilTagApproachNode._on_timer(harness)

    assert not published
    assert len(lost) == 1


def test_general_receipt_timeout_still_expires_duplicate_visual_sample(
    monkeypatch,
) -> None:
    harness, published, lost, receipt_now = duplicate_timer_harness(
        blind_enabled=False, receipt_now=10.36
    )
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)

    AprilTagApproachNode._on_timer(harness)

    assert not published
    assert len(lost) == 1


def test_approach_enable_event_resets_latched_session_state() -> None:
    resets = []
    harness = SimpleNamespace(
        _translation_filter=SimpleNamespace(reset=lambda: resets.append("translation")),
        _normal_filter=SimpleNamespace(reset=lambda: resets.append("normal")),
        _state_machine=SimpleNamespace(reset=lambda: resets.append("camera")),
        _base_state_machine=SimpleNamespace(reset=lambda: resets.append("base")),
        _active_tag_id=0,
        _last_valid_tag_x=0.3,
        _last_valid_receipt=10.0,
        _last_valid_yaw_error=0.0,
        _last_valid_cross_track=0.0,
        _last_fresh_final_observation_receipt=10.0,
        _final_approach_grace_eligible=True,
        _final_approach_grace_active=True,
        _blind_active=True,
        _blind_completed=True,
        _blind_planned_distance=0.05,
        _blind_start_odom=(0.0, 0.0, 0.0),
        _blind_previous_odom=(0.0, 0.0, 0.0),
        _blind_start_receipt=10.0,
        _last_odom_progress=0.01,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    harness._clear_final_sample = lambda: AprilTagApproachNode._clear_final_sample(
        harness
    )
    harness._clear_final_approach_grace = lambda: (
        AprilTagApproachNode._clear_final_approach_grace(harness)
    )
    harness._clear_blind_plan = lambda: AprilTagApproachNode._clear_blind_plan(
        harness
    )

    AprilTagApproachNode._on_approach_enabled(harness, Bool(data=True))

    assert resets == ["translation", "normal", "camera", "base"]
    assert harness._active_tag_id is None
    assert not harness._final_approach_grace_eligible
    assert not harness._blind_active
    assert not harness._blind_completed


def test_final_sample_stops_after_short_observation_hold(monkeypatch):
    harness, published, lost, receipt_now = duplicate_timer_harness(blind_enabled=False, receipt_now=10.30)
    calls = []
    harness._last_logged_base_state = ApproachState.FINAL_APPROACH
    harness._publish_final_approach_grace = lambda receipt: calls.append(receipt) or True
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)
    AprilTagApproachNode._on_timer(harness)
    assert calls == [pytest.approx(10.30)]
    assert not published


def test_expired_observation_is_not_extended_by_duplicate_tf(monkeypatch):
    harness, published, lost, receipt_now = duplicate_timer_harness(blind_enabled=False, receipt_now=10.50)
    monkeypatch.setattr(node_module.time, "monotonic", lambda: receipt_now)
    AprilTagApproachNode._on_timer(harness)
    assert lost
    assert not published
