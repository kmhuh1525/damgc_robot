"""Exact-time survivor map conversion contracts."""

from math import inf, nan
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformException

from rescue_robot_survivor.survivor_map_transform_node import (
    SurvivorMapTransformNode,
    transform_positions,
)


def make_positions():
    """Return two camera points with one shared image stamp."""
    message = PoseArray()
    message.header.frame_id = "camera_color_optical_frame"
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    for x, y, z in ((1.0, 0.0, 2.0), (0.0, 1.0, 3.0)):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def make_transform():
    """Use a 90-degree Z rotation plus translation."""
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "camera_color_optical_frame"
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.translation.z = 30.0
    transform.transform.rotation.z = 2**-0.5
    transform.transform.rotation.w = 2**-0.5
    return transform


def make_harness(transform=None):
    """Exercise the callback without starting a ROS graph."""
    publisher = Mock()
    buffer = Mock()
    buffer.lookup_transform.return_value = transform or make_transform()
    harness = SimpleNamespace(
        _target_frame="map",
        _tf_timeout_sec=0.6,
        _tf_buffer=buffer,
        _publisher=publisher,
        _warn=Mock(),
    )
    return harness


def test_multi_person_uses_one_exact_stamp_lookup_and_preserves_header():
    message = make_positions()
    harness = make_harness()

    SurvivorMapTransformNode._positions_callback(harness, message)

    harness._tf_buffer.lookup_transform.assert_called_once()
    args, kwargs = harness._tf_buffer.lookup_transform.call_args
    assert args[0] == "map"
    assert args[1] == message.header.frame_id
    assert args[2].nanoseconds == 123_000_000_456
    assert kwargs["timeout"].nanoseconds == 600_000_000
    harness._publisher.publish.assert_called_once()
    result = harness._publisher.publish.call_args.args[0]
    assert result.header.frame_id == "map"
    assert result.header.stamp == message.header.stamp
    assert len(result.poses) == 2
    assert [round(p.position.x, 6) for p in result.poses] == [10.0, 9.0]
    assert [round(p.position.y, 6) for p in result.poses] == [21.0, 20.0]
    assert [p.position.z for p in result.poses] == [32.0, 33.0]
    assert all(p.orientation.w == 1.0 for p in result.poses)


def test_empty_and_invalid_messages_never_lookup_or_publish():
    for mutate in (
        lambda m: m.poses.clear(),
        lambda m: setattr(m.header, "frame_id", ""),
        lambda m: (
            setattr(m.header.stamp, "sec", 0),
            setattr(m.header.stamp, "nanosec", 0),
        ),
        lambda m: setattr(m.poses[0].position, "x", nan),
        lambda m: setattr(m.poses[1].position, "z", inf),
    ):
        message = make_positions()
        mutate(message)
        harness = make_harness()
        SurvivorMapTransformNode._positions_callback(harness, message)
        harness._tf_buffer.lookup_transform.assert_not_called()
        harness._publisher.publish.assert_not_called()


def test_tf_failure_does_not_publish_or_try_latest():
    harness = make_harness()
    harness._tf_buffer.lookup_transform.side_effect = TransformException(
        "no transform at detection time"
    )

    SurvivorMapTransformNode._positions_callback(harness, make_positions())

    harness._tf_buffer.lookup_transform.assert_called_once()
    harness._publisher.publish.assert_not_called()
    harness._warn.assert_called_once()


def test_nonfinite_transform_output_is_rejected():
    transform = make_transform()
    transform.transform.translation.x = inf
    harness = make_harness(transform)

    SurvivorMapTransformNode._positions_callback(harness, make_positions())

    harness._tf_buffer.lookup_transform.assert_called_once()
    harness._publisher.publish.assert_not_called()
    harness._warn.assert_called_once()


def test_point_transform_does_not_use_camera_orientation():
    message = make_positions()
    message.poses[0].orientation.w = 0.0
    result = transform_positions(message, make_transform(), "map")
    assert result.poses[0].position.x == 10.0
    assert result.poses[0].orientation.w == 1.0
