"""Node-level tests with a fake model and no Ultralytics dependency."""

from collections import deque
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image

import rescue_robot_survivor.person_detector_node as detector_module
from rescue_robot_survivor.geometry_logic import CameraPoint


class FakeTensor:
    """Provide the tensor calls used by the model-result adapter."""

    def __init__(self, values):
        self._values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self._values


class FakeModel:
    """Return two people and one non-person in deliberately unsorted order."""

    def __init__(self):
        self.calls = []

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        boxes = SimpleNamespace(
            xyxy=FakeTensor(
                [[130, 20, 180, 100], [20, 20, 70, 100], [80, 20, 120, 100]]
            ),
            conf=FakeTensor([0.9, 0.8, 0.99]),
            cls=FakeTensor([0, 0, 56]),
        )
        return [SimpleNamespace(boxes=boxes)]


class RecordingPublisher:
    """Record published ROS messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def stamped_image(milliseconds):
    message = Image()
    message.header.stamp.sec, remainder = divmod(milliseconds, 1000)
    message.header.stamp.nanosec = remainder * 1_000_000
    return message


def matching_harness(size=5, slop=0.12):
    return SimpleNamespace(
        _depth_messages=deque(maxlen=size),
        _depth_lock=threading.Lock(),
        _depth_event=threading.Event(),
        _sync_queue_size=size,
        _sync_slop_sec=slop,
        _stamp_to_nanoseconds=detector_module.PersonDetectorNode._stamp_to_nanoseconds,
        _nearest_depth=lambda rgb: None,
        get_logger=Mock(),
    )


def add_depth(harness, milliseconds):
    message = stamped_image(milliseconds)
    detector_module.PersonDetectorNode._depth_callback(harness, message)
    return message


def match_depth(harness, milliseconds):
    harness._nearest_depth = lambda rgb: (
        detector_module.PersonDetectorNode._nearest_depth(harness, rgb)
    )
    return detector_module.PersonDetectorNode._get_matching_depth(
        harness, stamped_image(milliseconds)
    )


def test_depth_sync_exact_and_nearest_stamp():
    harness = matching_harness()
    first = add_depth(harness, 1000)
    second = add_depth(harness, 1066)
    assert match_depth(harness, 1000) is first
    assert match_depth(harness, 1050) is second


def test_depth_sync_rejects_outside_slop():
    harness = matching_harness()
    add_depth(harness, 1000)
    assert match_depth(harness, 1133) is None
    harness.get_logger().warning.assert_called_once()


def test_depth_sync_prunes_stale_and_limits_buffer():
    harness = matching_harness(size=3)
    stale = add_depth(harness, 1000)
    for stamp in (1500, 1533, 1566, 1600):
        add_depth(harness, stamp)
    assert len(harness._depth_messages) == 3
    assert stale not in harness._depth_messages
    assert match_depth(harness, 1000) is None


def test_depth_sync_late_out_of_order_frame_cannot_evict_newer_depth():
    harness = matching_harness(size=2)
    add_depth(harness, 1000)
    newest = add_depth(harness, 1033)
    add_depth(harness, 500)
    add_depth(harness, 1016)
    assert len(harness._depth_messages) == 2
    assert match_depth(harness, 1033) is newest


def test_depth_sync_waits_for_late_exact_frame():
    harness = matching_harness()
    older = add_depth(harness, 867)
    harness._nearest_depth = lambda rgb: (
        detector_module.PersonDetectorNode._nearest_depth(harness, rgb)
    )
    harness._get_matching_depth = lambda rgb: (
        detector_module.PersonDetectorNode._get_matching_depth(harness, rgb)
    )
    arriving = stamped_image(1000)
    timer = threading.Timer(
        0.02,
        lambda: detector_module.PersonDetectorNode._depth_callback(
            harness, arriving
        ),
    )
    timer.start()
    try:
        chosen = detector_module.PersonDetectorNode._wait_for_matching_depth(
            harness, stamped_image(1000), timeout_sec=0.1
        )
    finally:
        timer.join()
    assert chosen is arriving
    assert chosen is not older


def test_depth_sync_wait_timeout_rejects_old_frame():
    harness = matching_harness()
    add_depth(harness, 800)
    harness._nearest_depth = lambda rgb: (
        detector_module.PersonDetectorNode._nearest_depth(harness, rgb)
    )
    harness._get_matching_depth = lambda rgb: (
        detector_module.PersonDetectorNode._get_matching_depth(harness, rgb)
    )
    start = time.monotonic()
    assert detector_module.PersonDetectorNode._wait_for_matching_depth(
        harness, stamped_image(1000), timeout_sec=0.01
    ) is None
    assert time.monotonic() - start >= 0.01


def test_slow_inference_queue_keeps_only_latest_rgb():
    harness = SimpleNamespace(
        _pending_image_lock=threading.Lock(),
        _pending_image=None,
        _image_event=threading.Event(),
        _worker_shutdown=threading.Event(),
        _stamp_to_nanoseconds=detector_module.PersonDetectorNode._stamp_to_nanoseconds,
    )
    now_ms = time.time_ns() // 1_000_000
    first = stamped_image(now_ms - 50)
    latest = stamped_image(now_ms)
    detector_module.PersonDetectorNode._enqueue_image_callback(harness, first)
    detector_module.PersonDetectorNode._enqueue_image_callback(harness, latest)
    processed = []
    harness._image_callback = lambda msg: (
        processed.append(msg), harness._worker_shutdown.set()
    )
    detector_module.PersonDetectorNode._worker_loop(harness)
    assert processed == [latest]


def test_stale_rgb_is_discarded_before_inference_queue():
    harness = SimpleNamespace(
        _pending_image_lock=threading.Lock(),
        _pending_image=None,
        _image_event=threading.Event(),
        _stamp_to_nanoseconds=detector_module.PersonDetectorNode._stamp_to_nanoseconds,
    )
    old = stamped_image(time.time_ns() // 1_000_000 - 500)
    detector_module.PersonDetectorNode._enqueue_image_callback(harness, old)
    assert harness._pending_image is None
    assert not harness._image_event.is_set()


def test_model_is_loaded_once_and_source_header_is_preserved():
    model = FakeModel()
    debug_publisher = RecordingPublisher()
    positions_publisher = RecordingPublisher()
    parameter_publisher = RecordingPublisher()
    model_loads = []

    def fake_yolo(model_name):
        model_loads.append(model_name)
        return model

    def publisher_for(message_type, *_args, **_kwargs):
        if message_type is Image:
            return debug_publisher
        if message_type is PoseArray:
            return positions_publisher
        return parameter_publisher

    rclpy.init()
    node = None
    try:
        with (
            patch.object(
                detector_module,
                "torch",
                SimpleNamespace(
                    cuda=SimpleNamespace(is_available=lambda: False)
                ),
            ),
            patch.object(detector_module, "YOLO", side_effect=fake_yolo),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_publisher",
                side_effect=publisher_for,
            ),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_subscription",
                return_value=object(),
            ),
        ):
            node = detector_module.PersonDetectorNode()
            debug_publisher.messages.clear()
            positions_publisher.messages.clear()
            source = CvBridge().cv2_to_imgmsg(
                np.zeros((120, 200, 3), dtype=np.uint8), encoding="bgr8"
            )
            source.header.frame_id = "camera_color_optical_frame"
            source.header.stamp.sec = 123
            source.header.stamp.nanosec = 456

            node._image_callback(source)
            node._image_callback(source)

        assert model_loads == ["yolo11n.pt"]
        assert len(model.calls) == 2
        assert model.calls[0]["classes"] == [0]
        assert model.calls[0]["device"] == "cpu"
        assert len(debug_publisher.messages) == 2
        assert len(positions_publisher.messages) == 2
        output = debug_publisher.messages[0]
        assert output.header.frame_id == source.header.frame_id
        assert output.header.stamp == source.header.stamp
        annotated = CvBridge().imgmsg_to_cv2(output, desired_encoding="bgr8")
        assert np.count_nonzero(annotated) > 0
        assert positions_publisher.messages[0].poses == []
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_synthetic_depth_and_camera_info_publish_ordered_camera_positions():
    model = FakeModel()
    debug_publisher = RecordingPublisher()
    positions_publisher = RecordingPublisher()
    parameter_publisher = RecordingPublisher()

    def publisher_for(message_type, *_args, **_kwargs):
        if message_type is Image:
            return debug_publisher
        if message_type is PoseArray:
            return positions_publisher
        return parameter_publisher

    rclpy.init()
    node = None
    try:
        with (
            patch.object(
                detector_module,
                "torch",
                SimpleNamespace(
                    cuda=SimpleNamespace(is_available=lambda: False)
                ),
            ),
            patch.object(detector_module, "YOLO", return_value=model),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_publisher",
                side_effect=publisher_for,
            ),
            patch.object(
                detector_module.PersonDetectorNode,
                "create_subscription",
                return_value=object(),
            ),
        ):
            node = detector_module.PersonDetectorNode()
            debug_publisher.messages.clear()
            positions_publisher.messages.clear()

            camera_info = CameraInfo()
            camera_info.header.frame_id = "camera_color_optical_frame"
            camera_info.width = 200
            camera_info.height = 120
            camera_info.p = [
                100.0,
                0.0,
                100.0,
                0.0,
                0.0,
                100.0,
                60.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            node._camera_info_callback(camera_info)

            bridge = CvBridge()
            depth = bridge.cv2_to_imgmsg(
                np.full((120, 200), 2000, dtype=np.uint16),
                encoding="16UC1",
            )
            depth.header.frame_id = "camera_color_optical_frame"
            depth.header.stamp.sec = 123
            depth.header.stamp.nanosec = 456
            node._depth_callback(depth)

            source = bridge.cv2_to_imgmsg(
                np.zeros((120, 200, 3), dtype=np.uint8), encoding="bgr8"
            )
            source.header.frame_id = "camera_color_optical_frame"
            source.header.stamp.sec = 123
            source.header.stamp.nanosec = 456
            node._image_callback(source)

        assert len(debug_publisher.messages) == 1
        assert len(positions_publisher.messages) == 1
        positions = positions_publisher.messages[0]
        assert positions.header.stamp == source.header.stamp
        assert positions.header.frame_id == source.header.frame_id
        assert len(positions.poses) == 2
        assert positions.poses[0].position.x < 0.0
        assert positions.poses[1].position.x > 0.0
        assert positions.poses[0].position.z == 2.0
        assert positions.poses[1].position.z == 2.0
        assert all(pose.orientation.w == 1.0 for pose in positions.poses)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_pose_array_omits_invalid_person_without_origin_placeholder():
    positions_publisher = RecordingPublisher()
    node = SimpleNamespace(_positions_publisher=positions_publisher)
    source = Image()
    source.header.frame_id = "camera_color_optical_frame"
    source.header.stamp.sec = 42

    detector_module.PersonDetectorNode._publish_positions(
        node,
        {
            1: CameraPoint(-0.5, 0.1, 2.0),
            2: None,
            3: CameraPoint(0.6, 0.2, 3.0),
        },
        source,
    )

    positions = positions_publisher.messages[0]
    assert positions.header == source.header
    assert len(positions.poses) == 2
    assert [pose.position.x for pose in positions.poses] == [-0.5, 0.6]
    assert all(pose.orientation.w == 1.0 for pose in positions.poses)
