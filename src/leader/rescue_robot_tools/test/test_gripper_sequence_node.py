"""Regression tests for the existing Leader gripper sequence."""

from pathlib import Path
from types import SimpleNamespace

from std_msgs.msg import Bool, String

from gripper_sequence_node import GripperSequenceNode, SequenceState


def make_harness() -> SimpleNamespace:
    raw_commands = []
    gripper_commands = []
    statuses = []
    clock = {"now": 10.0}
    harness = SimpleNamespace(
        _enabled=True,
        _state=SequenceState.WAITING_FOR_TAG,
        _tag_detected=False,
        _alignment_state="TAG_LOST",
        _open_raw=1000,
        _close_raw=450,
        _close_wait=2.0,
        _lift_enabled=False,
        _lift_raw=-1,
        _lift_min_raw=260,
        _lift_max_raw=670,
        _lift_raw_valid=False,
        _lost_rx64_raw=600,
        _lost_rx28_raw=500,
        _tag_lost_idle_enabled=True,
        _deadline=None,
        _now=lambda: clock["now"],
        _publish_raw=lambda *values: raw_commands.append(values),
        _publish_gripper=gripper_commands.append,
        _publish_status=statuses.append,
    )
    harness.raw_commands = raw_commands
    harness.gripper_commands = gripper_commands
    harness.statuses = statuses
    harness.clock = clock
    return harness


def test_alignment_topic_defaults_to_authoritative_base_state() -> None:
    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "gripper_sequence.launch.py"

    assert '"alignment_topic", "/leader/base_alignment/state"' in (
        scripts_dir / "gripper_sequence_node.py"
    ).read_text(encoding="utf-8")
    assert 'default_value="/leader/base_alignment/state"' in launch_file.read_text(
        encoding="utf-8"
    )
    launch_source = launch_file.read_text(encoding="utf-8")
    node_source = (scripts_dir / "gripper_sequence_node.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("open_raw", default_value="950")' in launch_source
    assert 'self.declare_parameter("open_raw", 950)' in node_source
    assert 'DeclareLaunchArgument("close_wait", default_value="3.0")' in launch_source
    assert 'self.declare_parameter("close_wait", 3.0)' in node_source
    assert 'DeclareLaunchArgument("lift_enabled", default_value="false")' in launch_source
    assert 'DeclareLaunchArgument("tag_lost_idle_enabled", default_value="true")' in (
        launch_source
    )
    assert 'DeclareLaunchArgument("node_namespace", default_value="")' in launch_source
    assert 'namespace=LaunchConfiguration("node_namespace")' in launch_source


def test_detection_opens_once_and_only_aligned_closes() -> None:
    harness = make_harness()
    harness._lift_enabled = True
    harness._lift_raw = 520
    harness._lift_raw_valid = True

    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._detection_callback(harness, Bool(data=True))

    assert harness._state == SequenceState.OPENING
    assert harness.raw_commands == [(-1.0, 1000.0, -1.0, -1.0, 1.0)]

    for state in ("COARSE", "NEAR", "FINAL_YAW_ALIGN", "FINAL_APPROACH", "STABILIZING"):
        GripperSequenceNode._alignment_callback(harness, String(data=state))
        GripperSequenceNode._on_timer(harness)
        assert harness._state == SequenceState.OPENING

    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.CLOSING
    assert harness.raw_commands[-1] == (-1.0, 450.0, -1.0, -1.0, 1.0)
    assert harness._state == SequenceState.CLOSING
    assert harness._deadline == 12.0


def test_post_align_public_states_delay_close_until_final_aligned() -> None:
    harness = make_harness()
    harness._tag_lost_idle_enabled = False
    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    for detected, state in (
        (True, "STABILIZING"),
        (True, "FINAL_APPROACH"),
        (False, "FINAL_APPROACH"),
        (True, "FINAL_APPROACH"),
        (True, "STABILIZING"),
    ):
        GripperSequenceNode._detection_callback(harness, Bool(data=detected))
        GripperSequenceNode._alignment_callback(harness, String(data=state))
        GripperSequenceNode._on_timer(harness)
        assert harness._state == SequenceState.OPENING
        assert len(harness.raw_commands) == 1  # OPEN only

    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._on_timer(harness)
    assert [command[1] for command in harness.raw_commands] == [1000.0, 450.0]


def test_tag_loss_restores_safe_idle_pose_with_torque_enabled() -> None:
    harness = make_harness()
    harness._tag_detected = True
    harness._state = SequenceState.OPENING

    GripperSequenceNode._detection_callback(harness, Bool(data=False))

    assert harness.raw_commands == [(600.0, 500.0, -1.0, 1.0, 1.0)]
    assert harness.statuses[-1] == "TAG_LOST idle pose rx64=600 rx28=500"


def test_follower_tag_loss_policy_does_not_reposition_either_motor() -> None:
    harness = make_harness()
    harness._tag_detected = True
    harness._state = SequenceState.OPENING
    harness._tag_lost_idle_enabled = False

    GripperSequenceNode._detection_callback(harness, Bool(data=False))

    assert harness._tag_detected is False
    assert harness.raw_commands == []


def test_valid_lift_runs_once_after_close_wait() -> None:
    harness = make_harness()
    harness._lift_enabled = True
    harness._lift_raw = 300
    harness._lift_raw_valid = True
    harness._state = SequenceState.CLOSING
    harness._tag_detected = True
    harness._alignment_state = "ALIGNED"
    harness._deadline = 12.0

    harness.clock["now"] = 12.0
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.LIFTING
    assert harness.raw_commands == [(300.0, -1.0, -1.0, 1.0, -1.0)]
    assert harness.statuses[-1] == "LIFTING lift_raw=300"

    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness.raw_commands == [(300.0, -1.0, -1.0, 1.0, -1.0)]
    assert harness.statuses[-1] == "DONE lift_raw=300"
    assert not harness.gripper_commands


def test_follower_open_close_and_future_lift_are_each_one_shot() -> None:
    harness = make_harness()
    harness._open_raw = 950
    harness._close_raw = 350
    harness._lift_enabled = True
    harness._lift_raw = 520
    harness._lift_raw_valid = True

    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._on_timer(harness)

    harness.clock["now"] = 12.0
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._alignment_callback(harness, String(data="ALIGNED"))
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness.raw_commands == [
        (-1.0, 950.0, -1.0, -1.0, 1.0),
        (-1.0, 350.0, -1.0, -1.0, 1.0),
        (520.0, -1.0, -1.0, 1.0, -1.0),
    ]


def test_lift_disabled_completes_immediately_after_close() -> None:
    harness = make_harness()
    harness._state = SequenceState.OPENING
    harness._alignment_state = "ALIGNED"

    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness._deadline is None
    assert harness.raw_commands == [(-1.0, 450.0, -1.0, -1.0, 1.0)]


def test_invalid_enabled_lift_closes_but_never_lifts() -> None:
    harness = make_harness()
    harness._lift_enabled = True
    harness._lift_raw = -1
    harness._lift_raw_valid = False
    harness._state = SequenceState.OPENING
    harness._alignment_state = "ALIGNED"

    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness.raw_commands == [(-1.0, 450.0, -1.0, -1.0, 1.0)]
    assert harness.statuses[-1] == "DONE lift_skipped_invalid_raw"


def test_out_of_range_lift_closes_but_never_lifts() -> None:
    harness = make_harness()
    harness._lift_enabled = True
    harness._lift_raw = 776
    harness._lift_raw_valid = False
    harness._state = SequenceState.OPENING
    harness._alignment_state = "ALIGNED"

    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert harness.raw_commands == [(-1.0, 450.0, -1.0, -1.0, 1.0)]


def test_done_does_not_rearm_on_detection_flicker_while_aligned_latched() -> None:
    harness = make_harness()
    harness._state = SequenceState.DONE
    harness._alignment_state = "ALIGNED"

    GripperSequenceNode._detection_callback(harness, Bool(data=False))
    GripperSequenceNode._on_timer(harness)
    GripperSequenceNode._detection_callback(harness, Bool(data=True))
    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.DONE
    assert not harness.raw_commands
    assert not harness.gripper_commands


def test_done_rearms_only_after_detection_and_alignment_clear() -> None:
    harness = make_harness()
    harness._state = SequenceState.DONE
    harness._tag_detected = False
    harness._alignment_state = "TAG_LOST"

    GripperSequenceNode._on_timer(harness)

    assert harness._state == SequenceState.WAITING_FOR_TAG
    assert harness.statuses[-1] == "WAITING_FOR_TAG next cycle armed"
