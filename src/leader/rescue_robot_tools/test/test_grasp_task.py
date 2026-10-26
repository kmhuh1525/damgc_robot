"""Single-start grasp must reject stale distance and latch its completion."""
from types import SimpleNamespace as NS
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from gripper_sequence_node import GripperSequenceNode, SequenceState


def test_distance_requires_current_pose_and_camera_distance():
    pose = PoseStamped()
    pose.header.stamp.sec = 10
    pose.pose.position.z = .20
    h = NS(_distance_pose=pose, _distance_received=10., get_parameter=lambda name: NS(value={"grasp_distance": .20, "grasp_distance_tolerance": .02}[name]))
    assert GripperSequenceNode._task_distance_ok(h, 10.1)
    assert not GripperSequenceNode._task_distance_ok(h, 10.7)
    h._distance_received = 13.
    assert not GripperSequenceNode._task_distance_ok(h, 13.1)
    pose.header.stamp.sec = 13
    pose.pose.position.z = .30
    assert not GripperSequenceNode._task_distance_ok(h, 13.1)


def test_tag_reacquisition_does_not_open_after_done():
    h = NS(_enabled=True, _task_mode=True, _task_phase="DONE", _state=SequenceState.DONE, _tag_detected=False)
    GripperSequenceNode._detection_callback(h, Bool(data=True))
    assert h._tag_detected
    assert h._task_phase == "DONE"
    assert h._state == SequenceState.DONE
    GripperSequenceNode._task_timer(h)


def test_stop_before_close_keeps_perception_session_valid():
    queues = []
    h = NS(_queue_task_controls=lambda controls, phase: queues.append((controls, phase)))
    GripperSequenceNode._stop_task_controls(h, "CLOSE")
    assert queues == [([("guard", False), ("selector", "STOP")], "CLOSE")]


def test_close_rechecks_distance_after_base_stop():
    commands, faults = [], []
    h = NS(_task_phase="CLOSE", _task_started=9., _now=lambda:10., get_parameter=lambda _:NS(value=120.), _alignment_received=9.9, _alignment_state="ALIGNED", _tag_detected=True, _task_distance_ok=lambda _:False, _task_fault=faults.append, _publish_raw=lambda *a:commands.append(a))
    GripperSequenceNode._task_timer(h)
    assert faults
    assert not commands


def test_motor_error_stops_task_before_navigation_or_completion():
    faults = []
    h = NS(_task_phase="WAIT_OPEN", _task_fault=faults.append)
    GripperSequenceNode._task_motor_status(h, NS(data="ERROR serial write failed"))
    assert faults == ["motor: ERROR serial write failed"]


def test_motor_acknowledgment_must_match_gripper_target():
    h = NS(_task_phase="WAIT_CLOSE", _open_raw=1000, _close_raw=331, _motor_ack=False)
    GripperSequenceNode._task_motor_status(h, NS(data="OK rx64=-1 rx28=1000 rx64_torque=-1 rx28_torque=1"))
    assert not h._motor_ack
    GripperSequenceNode._task_motor_status(h, NS(data="OK rx64=-1 rx28=331 rx64_torque=-1 rx28_torque=1"))
    assert h._motor_ack


def test_missing_motor_acknowledgment_faults_before_open_wait_finishes():
    faults = []
    h = NS(_task_phase="WAIT_OPEN", _motor_ack=False, _motor_ack_deadline=9., _now=lambda:10., _task_fault=faults.append)
    GripperSequenceNode._task_timer(h)
    assert faults == ["motor command acknowledgment timeout"]


def test_republished_pose_does_not_refresh_original_receipt():
    pose = PoseStamped()
    pose.header.stamp.sec = 9
    h = NS(_distance_pose=None, _distance_received=0., _now=lambda:10.)
    GripperSequenceNode._task_pose_callback(h, pose)
    assert h._distance_received == 10.
    h._now = lambda:10.4
    GripperSequenceNode._task_pose_callback(h, pose)
    assert h._distance_received == 10.
