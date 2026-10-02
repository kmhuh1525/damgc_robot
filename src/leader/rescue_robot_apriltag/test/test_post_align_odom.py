"""Post-align final advance and its separation from the tag-loss fallback."""

from math import radians, sqrt
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.time import Time

from rescue_robot_apriltag.approach_logic import ApproachState
from rescue_robot_apriltag.apriltag_approach_node import AprilTagApproachNode
from rescue_robot_apriltag.base_alignment_logic import (
    AlignmentDecision,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def harness(*, enabled=True, tag_x=0.231):
    clock = {"now": 10.0}
    h = SimpleNamespace(
        _base_frame="base_link",
        _post_align_odom_enabled=enabled,
        _post_align_grasp_target_distance=0.16,
        _post_align_max_distance=0.12,
        _post_align_max_duration=8.0,
        _post_align_odom_timeout=0.25,
        _blind_last_tag_max_age=0.25,
        _blind_max_duration=5.0,
        _blind_planned_distance=0.04,
        _blind_active=False,
        _blind_completed=False,
        _last_valid_tag_x=0.27,
        _last_odom_progress=0.0,
        _final_target_distance=0.23,
        _last_odom=(0.0, 0.0, 0.0, 10.0, 10.0),
        _last_raw_zero_time=None,
        _last_guarded_zero_time=None,
        _last_fresh_final_observation_time=None,
        _final_approach_grace_eligible=False,
        _final_approach_grace_active=False,
        _normal_filter=SimpleNamespace(add=lambda x, y, _stamp: (x, y), reset=lambda: None),
        _translation_filter=SimpleNamespace(reset=lambda: None),
        _state_machine=SimpleNamespace(update=lambda *_args: ApproachState.TAG_LOST),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=int(clock["now"] * 1e9))),
        get_logger=lambda: SimpleNamespace(warning=lambda *_args, **_kwargs: None),
        _log_base_state_change=lambda _state: None,
    )
    h.clock = clock
    for name in (
        "_command_pub", "_control_target_pub", "_control_mode_pub", "_base_state_pub",
        "_blind_active_pub", "_last_valid_tag_x_pub", "_blind_distance_pub", "_odom_progress_pub",
        "_post_active_pub", "_post_distance_pub", "_post_progress_pub", "_post_tag_x_pub",
        "_base_pose_pub", "_base_forward_pub", "_base_lateral_pub", "_base_bearing_pub",
        "_normal_heading_pub", "_prealign_target_pub", "_final_target_pub",
        "_final_position_error_pub", "_final_yaw_error_pub", "_detected_pub", "_tag_id_pub", "_state_pub",
    ):
        setattr(h, name, Publisher())
    for name in (
        "_reset_post_align", "_publish_post_diagnostics", "_publish_post_command",
        "_abort_post_align", "_start_post_align", "_publish_post_cycle",
        "_publish_post_stop_cycle", "_publish_post_completed_cycle", "_fresh_odom",
        "_publish_atomic_command", "_publish_blind_diagnostics", "_publish_blind_command",
        "_publish_blind_cycle", "_publish_lost", "_publish_base_lost", "_on_timer",
        "_blind_plan",
        "_on_raw_velocity", "_on_guarded_velocity", "_make_target_pose", "_make_control_pose",
    ):
        setattr(h, name, getattr(AprilTagApproachNode, name).__get__(h))
    h._reset_post_align()
    h._base_state_machine = SimpleNamespace(
        update=lambda *_args: AlignmentDecision(ApproachState.ALIGNED, ControlMode.ALIGNED),
        reset=lambda: None,
    )
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_color_optical_frame"
    transform.transform.rotation.w = 1.0
    h._tf_buffer = SimpleNamespace(lookup_transform=lambda *_args, **_kwargs: transform)
    h._tag_timeout = 1.0
    h._tf_lookup_timeout = 0.0
    h._pre_align_distance = 0.30
    h._active_tag_id = 0
    pose = PoseStamped()
    pose.header.frame_id = "camera_color_optical_frame"
    pose.header.stamp = Time(nanoseconds=10_000_000_000).to_msg()
    pose.pose.position.x = tag_x
    pose.pose.orientation.y = -sqrt(0.5)
    pose.pose.orientation.w = sqrt(0.5)
    h.pose = pose
    return h


def last_command(h):
    message = h._command_pub.messages[-1]
    return message.alignment_state, message.control_mode


def test_disabled_visual_alignment_publishes_existing_aligned():
    h = harness(enabled=False)
    AprilTagApproachNode._publish_base_outputs(h, h.pose)
    assert last_command(h) == ("ALIGNED", "ALIGNED")
    assert h._base_state_pub.messages[-1].data == "ALIGNED"
    assert not h._post_align_active


def test_post_odom_timeout_is_independent_of_blind_freshness():
    h = harness()
    h._last_odom = (0.0, 0.0, 0.0, 10.0, 10.0)
    assert h._fresh_odom(10.3) is None
    assert h._fresh_odom(10.3, max_age=0.4) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    ("tag_x", "expected"), [(0.231, 0.071), (0.220, 0.060)]
)
def test_visual_alignment_starts_bounded_post_advance_without_public_aligned(
    tag_x, expected
):
    h = harness(tag_x=tag_x)
    AprilTagApproachNode._publish_base_outputs(h, h.pose)
    assert h._post_align_active
    assert h._post_align_planned_distance == pytest.approx(expected)
    assert h._post_align_start_tag_x == pytest.approx(tag_x)
    assert last_command(h) == ("FINAL_APPROACH", "BLIND_FINAL_APPROACH")
    assert h._blind_active_pub.messages[-1].data is False
    assert h._post_active_pub.messages[-1].data is True
    assert all(message.data != "ALIGNED" for message in h._base_state_pub.messages)


@pytest.mark.parametrize("tag_sequence", [(True, True), (False, False), (False, True)])
def test_tag_visibility_never_cancels_active_post_plan(tag_sequence):
    h = harness()
    h._start_post_align(0.23, 10.0)
    h._candidate_ids = lambda: pytest.fail("post-align must not inspect tags")
    for i, _tag_visible in enumerate(tag_sequence, start=1):
        h.clock["now"] = 10.0 + 0.05 * i
        h._last_odom = (0.01 * i, 0.0, 0.0, h.clock["now"], h.clock["now"])
        h._on_timer()
        assert h._post_align_active
        assert last_command(h) == ("FINAL_APPROACH", "BLIND_FINAL_APPROACH")


def test_completion_waits_for_new_zero_raw_and_guarded_commands():
    h = harness()
    h._start_post_align(0.23, 10.0)
    h.clock["now"] = 10.05
    h._last_odom = (0.04, 0.0, 0.0, 10.05, 10.05)
    h._publish_post_cycle(10.05)
    h.clock["now"] = 10.1
    h._last_odom = (0.071, 0.0, 0.0, 10.1, 10.1)
    h._publish_post_cycle(10.1)
    assert h._post_align_stopping and not h._post_align_completed
    assert h._post_align_active
    assert h._post_active_pub.messages[-1].data is True
    assert last_command(h) == ("STABILIZING", "STABILIZING")
    h._publish_lost(10.105)
    assert h._post_stop_started == pytest.approx(10.1)
    h.clock["now"] = 10.11
    h._on_raw_velocity(Twist())
    h._publish_post_stop_cycle(10.11)
    assert not h._post_align_completed
    h.clock["now"] = 10.12
    h._on_guarded_velocity(Twist())
    h._publish_post_stop_cycle(10.12)
    assert h._post_align_completed and not h._post_align_active
    assert h._post_active_pub.messages[-1].data is False
    assert last_command(h) == ("ALIGNED", "ALIGNED")
    h._candidate_ids = lambda: pytest.fail("completed mission must not select a tag")
    h._on_timer()
    assert h._post_align_completed
    assert h._post_align_planned_distance == pytest.approx(0.07)


@pytest.mark.parametrize("tag_x", [float("nan"), 0.15, 0.40])
def test_invalid_visual_plan_aborts_without_aligned(tag_x):
    h = harness()
    assert not h._start_post_align(tag_x, 10.0)
    assert last_command(h) == ("TAG_LOST", "TAG_LOST")
    assert not h._post_align_completed


@pytest.mark.parametrize(
    "sample,time_now",
    [
        (None, 10.1),
        ((float("nan"), 0.0, 0.0, 10.1, 10.1), 10.1),
        ((0.01, 0.0, 0.0, 10.1, 10.1), 10.4),
        ((0.06, 0.0, 0.0, 10.1, 10.1), 10.1),
        ((-0.02, 0.0, 0.0, 10.1, 10.1), 10.1),
        ((0.01, 0.031, 0.0, 10.1, 10.1), 10.1),
        ((0.01, 0.0, radians(13), 10.1, 10.1), 10.1),
        ((0.01, 0.0, 0.0, 18.01, 18.01), 18.01),
    ],
)
def test_odom_faults_abort_and_never_publish_aligned(sample, time_now):
    h = harness()
    h._start_post_align(0.23, 10.0)
    h._last_odom = sample
    h.clock["now"] = time_now
    h._publish_post_cycle(time_now)
    assert not h._post_align_active and not h._post_align_completed
    assert last_command(h) == ("TAG_LOST", "TAG_LOST")
    assert all(message.data != "ALIGNED" for message in h._base_state_pub.messages)


def test_abort_clears_private_aligned_latch_before_next_timer():
    h = harness()
    machine = BaseAlignmentStateMachine(BaseAlignmentThresholds(
        orientation_engage_distance=0.40, orientation_disengage_distance=0.43,
        turn_enter_error_deg=8.0, turn_exit_error_deg=3.0,
        tag_recenter_enter_deg=18.0, tag_recenter_exit_deg=11.0,
        near_normal_correction_limit_deg=6.0, pre_align_position_tolerance=0.02,
        final_position_tolerance=0.02, final_yaw_tolerance_deg=5.0,
        final_realign_yaw_error_deg=8.0, stable_time=0.30, sample_timeout=1.0,
    ))
    machine._aligned_latched = True
    machine._active_tag_id = 0
    h._base_state_machine = machine
    h._start_post_align(0.23, 10.0)
    h._last_odom = None
    h._publish_post_cycle(10.1)
    assert not machine._aligned_latched
    h._publish_base_lost(10.2)
    assert last_command(h) == ("TAG_LOST", "TAG_LOST")
    assert h._last_valid_tag_x is None
    assert h._blind_plan(10.2) is None


def test_no_downstream_zero_confirmation_aborts_without_aligned():
    h = harness()
    h._start_post_align(0.23, 10.0)
    h.clock["now"] = 10.05
    h._last_odom = (0.04, 0.0, 0.0, 10.05, 10.05)
    h._publish_post_cycle(10.05)
    h.clock["now"] = 10.10
    h._last_odom = (0.071, 0.0, 0.0, 10.10, 10.10)
    h._publish_post_cycle(10.10)
    assert h._post_align_stopping
    h.clock["now"] = 18.01
    h._publish_post_stop_cycle(18.01)
    assert not h._post_align_completed
    assert last_command(h) == ("TAG_LOST", "TAG_LOST")


def test_explicit_new_mission_reset_discards_post_plan_and_latch():
    h = harness()
    h._start_post_align(0.23, 10.0)
    h._post_align_completed = True
    h._reset_post_align()
    assert not h._post_align_active
    assert not h._post_align_completed
    assert h._post_align_start_odom is None
    assert h._post_align_previous_odom is None
    assert h._post_align_start_time is None
    assert h._post_align_planned_distance == 0.0
    assert h._post_align_progress == 0.0
    assert h._post_align_start_tag_x == 0.0


@pytest.mark.parametrize("enabled", [False, True])
def test_blind_completion_only_hands_off_when_enabled(enabled):
    h = harness(enabled=enabled)
    h._blind_active = True
    h._blind_start_time = 10.0
    h._blind_start_odom = (0.0, 0.0, 0.0)
    h._blind_previous_odom = (0.03, 0.0, 0.0)
    h._last_odom = (0.04, 0.0, 0.0, 10.1, 10.1)
    h.clock["now"] = 10.1
    h._publish_blind_cycle(10.1)
    if enabled:
        assert h._post_align_active and not h._blind_completed
        assert h._post_align_planned_distance == pytest.approx(0.07)
        assert last_command(h) == ("FINAL_APPROACH", "BLIND_FINAL_APPROACH")
        assert h._blind_active_pub.messages[-1].data is False
        h.clock["now"] = 10.15
        h._last_odom = (0.08, 0.0, 0.0, 10.15, 10.15)
        h._publish_post_cycle(10.15)
        h.clock["now"] = 10.20
        h._last_odom = (0.111, 0.0, 0.0, 10.20, 10.20)
        h._publish_post_cycle(10.20)
        assert last_command(h) == ("STABILIZING", "STABILIZING")
        h.clock["now"] = 10.21
        h._on_raw_velocity(Twist())
        h._on_guarded_velocity(Twist())
        h._publish_post_stop_cycle(10.21)
        assert h._post_align_completed
        assert last_command(h) == ("ALIGNED", "ALIGNED")
    else:
        assert h._blind_completed
        assert last_command(h) == ("ALIGNED", "ALIGNED")
