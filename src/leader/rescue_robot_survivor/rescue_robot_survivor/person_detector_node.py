"""ROS 2 node that publishes YOLO11n detections with aligned depth."""

import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image

from rescue_robot_survivor.detection_logic import (
    Detection,
    draw_person_detections,
    prepare_person_detections,
)
from rescue_robot_survivor.depth_logic import estimate_person_distance
from rescue_robot_survivor.geometry_logic import (
    CameraIntrinsics,
    CameraPoint,
    deproject_pixel_to_camera_xyz,
    get_rectified_intrinsics,
    roi_center_pixel,
)

try:
    import torch
except ImportError:  # Optional for package build and pure tests.
    torch = None

try:
    from ultralytics import YOLO
except ImportError:  # Reported as an actionable startup error.
    YOLO = None


IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

CAMERA_INFO_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

POSITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class PersonDetectorNode(Node):
    """Detect every COCO person and publish a frame-local annotated image."""

    def __init__(self) -> None:
        """Initialize parameters, the model, and ROS endpoints once."""
        super().__init__("person_detector")
        self._image_topic = str(
            self.declare_parameter(
                "image_topic", "/leader/camera/color/image_rect"
            ).value
        )
        self._debug_image_topic = str(
            self.declare_parameter(
                "debug_image_topic", "/leader/survivor/debug_image"
            ).value
        )
        self._model_name = str(
            self.declare_parameter("model_name", "yolo11n.pt").value
        )
        self._confidence_threshold = float(
            self.declare_parameter("confidence_threshold", 0.5).value
        )
        self._requested_device = str(
            self.declare_parameter("device", "auto").value
        ).strip()
        self._aligned_depth_topic = str(
            self.declare_parameter(
                "aligned_depth_topic",
                "/leader/camera/aligned_depth_to_color/image_raw",
            ).value
        )
        self._depth_roi_width_ratio = float(
            self.declare_parameter("depth_roi_width_ratio", 0.25).value
        )
        self._depth_roi_height_ratio = float(
            self.declare_parameter("depth_roi_height_ratio", 0.25).value
        )
        self._min_depth_m = float(
            self.declare_parameter("min_depth_m", 0.2).value
        )
        self._max_depth_m = float(
            self.declare_parameter("max_depth_m", 6.0).value
        )
        self._min_valid_depth_pixels = int(
            self.declare_parameter("min_valid_depth_pixels", 20).value
        )
        self._depth_scale_m_per_unit = float(
            self.declare_parameter("depth_scale_m_per_unit", 0.001).value
        )
        self._show_depth_roi = bool(
            self.declare_parameter("show_depth_roi", False).value
        )
        self._sync_slop_sec = float(
            self.declare_parameter("sync_slop_sec", 0.05).value
        )
        self._sync_queue_size = int(
            self.declare_parameter("sync_queue_size", 5).value
        )
        self._camera_info_topic = str(
            self.declare_parameter(
                "camera_info_topic", "/leader/camera/color/camera_info"
            ).value
        )
        self._camera_positions_topic = str(
            self.declare_parameter(
                "camera_positions_topic",
                "/leader/survivor/camera_positions",
            ).value
        )
        self._show_camera_xyz = bool(
            self.declare_parameter("show_camera_xyz", True).value
        )

        self._validate_parameters()
        if torch is None:
            raise RuntimeError(
                "PyTorch is not installed. Install the NVIDIA "
                "JetPack-compatible "
                "PyTorch build; do not replace it with a generic PyPI wheel."
            )
        if YOLO is None:
            raise RuntimeError(
                "Ultralytics is not installed. Prepare the Jetson-compatible "
                "PyTorch environment first, then install Ultralytics without "
                "allowing it to replace torch or torchvision."
            )

        self._device = self._select_device(self._requested_device)
        self._auto_cpu_fallback_used = False
        self._bridge = CvBridge()
        # Model construction happens exactly once, never in the image callback.
        self._model = YOLO(self._model_name)
        self._depth_messages = deque(maxlen=self._sync_queue_size)
        self._depth_lock = threading.Lock()
        self._depth_event = threading.Event()
        self._pending_image_lock = threading.Lock()
        self._pending_image = None
        self._image_event = threading.Event()
        self._worker_shutdown = threading.Event()
        self._image_callbacks = MutuallyExclusiveCallbackGroup()
        self._sensor_callbacks = MutuallyExclusiveCallbackGroup()
        self._camera_intrinsics: Optional[CameraIntrinsics] = None
        self._camera_frame_id = ""
        self._camera_info_lock = threading.Lock()
        self._publisher = self.create_publisher(
            Image, self._debug_image_topic, IMAGE_QOS
        )
        self._positions_publisher = self.create_publisher(
            PoseArray, self._camera_positions_topic, POSITION_QOS
        )
        self._image_subscription = self.create_subscription(
            Image, self._image_topic, self._enqueue_image_callback, IMAGE_QOS,
            callback_group=self._image_callbacks,
        )
        self._depth_subscription = self.create_subscription(
            Image, self._aligned_depth_topic, self._depth_callback, IMAGE_QOS,
            callback_group=self._sensor_callbacks,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            CAMERA_INFO_QOS,
            callback_group=self._sensor_callbacks,
        )

        self.get_logger().info(f"Model: {self._model_name}")
        self.get_logger().info(f"Input image topic: {self._image_topic}")
        self.get_logger().info(
            f"Debug image topic: {self._debug_image_topic}"
        )
        self.get_logger().info(
            f"Aligned depth topic: {self._aligned_depth_topic}"
        )
        self.get_logger().info(f"CameraInfo topic: {self._camera_info_topic}")
        self.get_logger().info(
            f"Camera positions topic: {self._camera_positions_topic}"
        )
        self.get_logger().info("Rectified projection source: CameraInfo.P")
        self.get_logger().info(
            f"Confidence threshold: {self._confidence_threshold:.3f}"
        )
        self.get_logger().info(f"Selected device: {self._device}")
        # Inference runs off the ROS executor so depth and incoming RGB keep
        # arriving while one frame is processed. Only the latest RGB is queued.
        self._worker = threading.Thread(
            target=self._worker_loop, name="person_detector_inference", daemon=True
        )
        self._worker.start()

    def destroy_node(self):
        self._worker_shutdown.set()
        self._image_event.set()
        self._depth_event.set()
        self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _enqueue_image_callback(self, message: Image) -> None:
        stamp = self._stamp_to_nanoseconds(message)
        # DDS may deliver an old RGB after a temporary CPU/GPU stall. Skip it
        # before inference rather than pairing it with a newer depth frame.
        if stamp and time.time_ns() - stamp > 250_000_000:
            return
        with self._pending_image_lock:
            self._pending_image = message
            self._image_event.set()

    def _worker_loop(self) -> None:
        while not self._worker_shutdown.is_set():
            self._image_event.wait(timeout=0.1)
            if self._worker_shutdown.is_set():
                break
            with self._pending_image_lock:
                message = self._pending_image
                self._pending_image = None
                self._image_event.clear()
            if message is not None:
                self._image_callback(message)

    def _validate_parameters(self) -> None:
        if not self._image_topic:
            raise ValueError("image_topic must not be empty")
        if not self._debug_image_topic:
            raise ValueError("debug_image_topic must not be empty")
        if not self._model_name:
            raise ValueError("model_name must not be empty")
        if not 0.0 <= self._confidence_threshold <= 1.0:
            raise ValueError(
                "confidence_threshold must be between 0.0 and 1.0"
            )
        if not self._requested_device:
            raise ValueError("device must not be empty")
        if not self._aligned_depth_topic:
            raise ValueError("aligned_depth_topic must not be empty")
        if not 0.0 < self._depth_roi_width_ratio <= 1.0:
            raise ValueError("depth_roi_width_ratio must be in (0.0, 1.0]")
        if not 0.0 < self._depth_roi_height_ratio <= 1.0:
            raise ValueError("depth_roi_height_ratio must be in (0.0, 1.0]")
        if self._min_depth_m < 0.0 or self._max_depth_m <= self._min_depth_m:
            raise ValueError("depth range is invalid")
        if self._min_valid_depth_pixels <= 0:
            raise ValueError("min_valid_depth_pixels must be positive")
        if self._depth_scale_m_per_unit <= 0.0:
            raise ValueError("depth_scale_m_per_unit must be positive")
        if self._sync_slop_sec <= 0.0:
            raise ValueError("sync_slop_sec must be positive")
        if self._sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")
        if not self._camera_info_topic:
            raise ValueError("camera_info_topic must not be empty")
        if not self._camera_positions_topic:
            raise ValueError("camera_positions_topic must not be empty")

    @staticmethod
    def _select_device(requested_device: str) -> str:
        if requested_device.lower() != "auto":
            return requested_device
        return "0" if torch.cuda.is_available() else "cpu"

    def _predict(self, image: Any) -> Any:
        try:
            return self._model.predict(
                source=image,
                conf=self._confidence_threshold,
                classes=[0],
                device=self._device,
                verbose=False,
            )
        except Exception:
            if (
                self._requested_device.lower() == "auto"
                and self._device != "cpu"
                and not self._auto_cpu_fallback_used
            ):
                self._auto_cpu_fallback_used = True
                self._device = "cpu"
                self.get_logger().warning(
                    "CUDA inference failed in auto mode; retrying on CPU."
                )
                return self._model.predict(
                    source=image,
                    conf=self._confidence_threshold,
                    classes=[0],
                    device=self._device,
                    verbose=False,
                )
            raise

    @staticmethod
    def _extract_detections(results: Any) -> List[Detection]:
        if not results or results[0].boxes is None:
            return []
        boxes = results[0].boxes
        coordinates = boxes.xyxy.detach().cpu().tolist()
        confidences = boxes.conf.detach().cpu().tolist()
        classes = boxes.cls.detach().cpu().tolist()
        return [
            Detection(*coordinate, float(confidence), int(class_id))
            for coordinate, confidence, class_id in zip(
                coordinates, confidences, classes
            )
        ]

    def _publish_image(self, image: Any, source_message: Image) -> None:
        debug_message = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_message.header = source_message.header
        self._publisher.publish(debug_message)

    @staticmethod
    def _stamp_to_nanoseconds(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _depth_callback(self, message: Image) -> None:
        """Keep the newest valid stamps, even if depth arrives out of order."""
        stamp = self._stamp_to_nanoseconds(message)
        if stamp == 0:
            return
        with self._depth_lock:
            messages = list(self._depth_messages)
            messages.append(message)
            newest = max(self._stamp_to_nanoseconds(item) for item in messages)
            # Bound retained age as well as count; never let a late old frame
            # displace a newer one from the synchronization cache.
            cutoff = newest - 500_000_000
            messages = [
                item for item in messages
                if self._stamp_to_nanoseconds(item) >= cutoff
            ]
            messages.sort(key=self._stamp_to_nanoseconds)
            self._depth_messages = deque(
                messages[-self._sync_queue_size:], maxlen=self._sync_queue_size
            )
        self._depth_event.set()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Cache valid rectified intrinsics from CameraInfo.P."""
        intrinsics = get_rectified_intrinsics(
            message.p,
            message.width,
            message.height,
            message.width,
            message.height,
        )
        if intrinsics is None or not message.header.frame_id:
            self.get_logger().warning(
                "CameraInfo has invalid P intrinsics, dimensions, or frame; "
                "using XYZ N/A.",
                throttle_duration_sec=5.0,
            )
            return
        with self._camera_info_lock:
            self._camera_intrinsics = intrinsics
            self._camera_frame_id = message.header.frame_id

    def _nearest_depth(self, rgb_message: Image):
        with self._depth_lock:
            depth_messages = tuple(self._depth_messages)
        if not depth_messages:
            return None
        rgb_stamp = self._stamp_to_nanoseconds(rgb_message)
        if rgb_stamp == 0:
            return None
        matching = [
            (abs(rgb_stamp - self._stamp_to_nanoseconds(depth)), depth)
            for depth in depth_messages
            if self._stamp_to_nanoseconds(depth) != 0
        ]
        if not matching:
            return None
        return min(matching, key=lambda item: item[0])

    def _get_matching_depth(self, rgb_message: Image):
        rgb_stamp = self._stamp_to_nanoseconds(rgb_message)
        nearest = self._nearest_depth(rgb_message)
        if nearest is None:
            return None
        delta_ns, depth_message = nearest
        delta_sec = delta_ns / 1e9
        if delta_sec > self._sync_slop_sec:
            signed_sec = (
                self._stamp_to_nanoseconds(depth_message) - rgb_stamp
            ) / 1e9
            self.get_logger().warning(
                "RGB/aligned depth timestamp difference is "
                f"{delta_sec:.3f}s (depth minus RGB {signed_sec:+.3f}s, "
                f"RGB age {(time.time_ns() - rgb_stamp) / 1e9:.3f}s); "
                "using N/A.",
                throttle_duration_sec=5.0,
            )
            return None
        return depth_message

    def _wait_for_matching_depth(self, rgb_message: Image, timeout_sec=0.15):
        """Allow an aligned depth callback to catch up without widening slop."""
        deadline = time.monotonic() + timeout_sec
        preferred_delta_ns = int(min(self._sync_slop_sec, 0.04) * 1e9)
        while True:
            self._depth_event.clear()
            nearest = self._nearest_depth(rgb_message)
            if nearest is not None and nearest[0] <= preferred_delta_ns:
                return nearest[1]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return self._get_matching_depth(rgb_message)
            self._depth_event.wait(remaining)

    def _estimate_distances(
        self, people, rgb_message: Image, rgb_shape, depth_message
    ):
        distances = {number: None for number in range(1, len(people) + 1)}
        rois = {}
        if depth_message is None:
            return distances, rois
        try:
            depth_image = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.get_logger().warning(
                f"Aligned depth conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        if depth_image.ndim != 2:
            self.get_logger().warning(
                f"Aligned depth must be single-channel, got {depth_image.shape}.",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        if depth_image.shape[:2] != rgb_shape[:2]:
            self.get_logger().warning(
                f"RGB/depth resolution mismatch: RGB {rgb_shape[:2]}, "
                f"depth {depth_image.shape[:2]}; using N/A.",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        if (
            depth_message.header.frame_id
            and rgb_message.header.frame_id
            and depth_message.header.frame_id != rgb_message.header.frame_id
        ):
            self.get_logger().warning(
                "RGB/aligned depth frame mismatch: "
                f"{rgb_message.header.frame_id} vs "
                f"{depth_message.header.frame_id}; using N/A.",
                throttle_duration_sec=5.0,
            )
            return distances, rois
        for number, person in enumerate(people, start=1):
            try:
                estimate, roi = estimate_person_distance(
                    depth_image,
                    depth_message.encoding,
                    (person.x1, person.y1, person.x2, person.y2),
                    self._depth_roi_width_ratio,
                    self._depth_roi_height_ratio,
                    self._depth_scale_m_per_unit,
                    self._min_depth_m,
                    self._max_depth_m,
                    self._min_valid_depth_pixels,
                )
            except (TypeError, ValueError) as error:
                self.get_logger().warning(
                    f"Depth estimate failed: {error}",
                    throttle_duration_sec=5.0,
                )
                continue
            distances[number] = estimate.distance_m
            if roi is not None:
                rois[number] = roi
        return distances, rois

    def _estimate_camera_points(
        self,
        distances: Dict[int, Optional[float]],
        rois: Dict[int, Tuple[int, int, int, int]],
        rgb_message: Image,
        rgb_shape,
    ) -> Dict[int, Optional[CameraPoint]]:
        points = {number: None for number in distances}
        with self._camera_info_lock:
            intrinsics = self._camera_intrinsics
            camera_frame_id = self._camera_frame_id
        if intrinsics is None:
            return points
        if (intrinsics.height, intrinsics.width) != rgb_shape[:2]:
            self.get_logger().warning(
                "RGB/CameraInfo resolution mismatch: RGB "
                f"{rgb_shape[:2]}, CameraInfo "
                f"{(intrinsics.height, intrinsics.width)}; using XYZ N/A.",
                throttle_duration_sec=5.0,
            )
            return points
        if not rgb_message.header.frame_id or (
            camera_frame_id != rgb_message.header.frame_id
        ):
            self.get_logger().warning(
                "RGB/CameraInfo frame mismatch or empty frame: "
                f"{rgb_message.header.frame_id} vs {camera_frame_id}; "
                "using XYZ N/A.",
                throttle_duration_sec=5.0,
            )
            return points
        for number, distance in distances.items():
            if distance is None or number not in rois:
                continue
            pixel = roi_center_pixel(rois[number])
            if pixel is None:
                continue
            points[number] = deproject_pixel_to_camera_xyz(
                pixel[0], pixel[1], distance, intrinsics
            )
        return points

    def _publish_positions(
        self,
        camera_points: Dict[int, Optional[CameraPoint]],
        source_message: Image,
    ) -> None:
        positions = PoseArray()
        positions.header = source_message.header
        for number in sorted(camera_points):
            point = camera_points[number]
            if point is None:
                continue
            pose = Pose()
            pose.position.x = point.x
            pose.position.y = point.y
            pose.position.z = point.z
            pose.orientation.w = 1.0
            positions.poses.append(pose)
        self._positions_publisher.publish(positions)

    def _image_callback(self, message: Image) -> None:
        camera_points: Dict[int, Optional[CameraPoint]] = {}
        # Pair at callback entry, before YOLO can outlast the five-frame cache.
        depth_message = self._wait_for_matching_depth(message)
        try:
            image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"RGB image conversion failed: {error}",
                throttle_duration_sec=5.0,
            )
            return

        try:
            results = self._predict(image)
            detections = self._extract_detections(results)
            people = prepare_person_detections(
                detections,
                self._confidence_threshold,
                image.shape[1],
                image.shape[0],
            )
            # Depth is optional: RGB detection and debug output remain alive
            # while the aligned stream is absent or temporarily mismatched.
            distances, rois = self._estimate_distances(
                people, message, image.shape, depth_message
            )
            camera_points = self._estimate_camera_points(
                distances, rois, message, image.shape
            )
            draw_person_detections(
                image,
                people,
                distances=distances,
                camera_points={
                    number: (
                        (point.x, point.y, point.z)
                        if point is not None
                        else None
                    )
                    for number, point in camera_points.items()
                },
                rois=rois,
                show_depth_roi=self._show_depth_roi,
                show_camera_xyz=self._show_camera_xyz,
            )
        except Exception as error:
            self.get_logger().error(
                f"Person frame processing failed: {error}",
                throttle_duration_sec=5.0,
            )
        self._publish_positions(camera_points, message)
        self._publish_image(image, message)


def main(args=None) -> None:
    """Run the person detector until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = PersonDetectorNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        rclpy.logging.get_logger("person_detector").fatal(str(error))
        raise SystemExit(1) from error
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
