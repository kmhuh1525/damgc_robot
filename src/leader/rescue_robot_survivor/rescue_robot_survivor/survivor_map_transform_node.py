"""Transform survivor camera positions into a target frame at image time."""

from math import isfinite

from geometry_msgs.msg import PointStamped, Pose, PoseArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


POSITION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def transform_positions(message, transform, target_frame):
    """Apply one timestamped TF to all valid camera-frame positions."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    if not all(
        isfinite(value)
        for value in (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
    ) or not (
        rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2
    ) > 0.0:
        raise ValueError("TF contains an invalid translation or rotation")
    result = PoseArray()
    result.header.stamp = message.header.stamp
    result.header.frame_id = target_frame
    for camera_pose in message.poses:
        point = PointStamped()
        point.header = message.header
        point.point = camera_pose.position
        mapped = do_transform_point(point, transform).point
        if not all(
            isfinite(value) for value in (mapped.x, mapped.y, mapped.z)
        ):
            raise ValueError("TF produced a non-finite map position")
        map_pose = Pose()
        map_pose.position = mapped
        map_pose.orientation.w = 1.0
        result.poses.append(map_pose)
    return result


class SurvivorMapTransformNode(Node):
    """Convert a PoseArray without changing the detector or TF ownership."""

    def __init__(self):
        super().__init__("survivor_map_transform")
        self._input_topic = str(
            self.declare_parameter(
                "input_topic", "/leader/survivor/camera_positions"
            ).value
        ).strip()
        self._output_topic = str(
            self.declare_parameter(
                "output_topic", "/leader/survivor/map_positions"
            ).value
        ).strip()
        self._target_frame = str(
            self.declare_parameter("target_frame", "map").value
        ).strip()
        self._tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.6).value
        )
        if not self._input_topic or not self._output_topic:
            raise ValueError("input_topic and output_topic must not be empty")
        if self._input_topic == self._output_topic:
            raise ValueError("input_topic and output_topic must differ")
        if not self._target_frame:
            raise ValueError("target_frame must not be empty")
        if not isfinite(self._tf_timeout_sec) or self._tf_timeout_sec <= 0.0:
            raise ValueError("tf_timeout_sec must be finite and positive")

        self._tf_buffer = Buffer()
        # A dedicated listener node keeps receiving TF during a timed lookup.
        self._tf_listener = TransformListener(
            self._tf_buffer, None, spin_thread=True
        )
        self._publisher = self.create_publisher(
            PoseArray, self._output_topic, POSITION_QOS
        )
        self._subscription = self.create_subscription(
            PoseArray, self._input_topic, self._positions_callback,
            POSITION_QOS,
        )
        self.get_logger().info(
            f"Transforming {self._input_topic} to {self._output_topic} "
            f"in {self._target_frame} at the input image timestamp"
        )

    def _positions_callback(self, message):
        if not message.poses:
            # The detector publishes an empty array when no XYZ is valid.
            return
        if not message.header.frame_id.strip():
            self._warn("Skipping positions with an empty source frame")
            return
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000 or (
            stamp.sec == 0 and stamp.nanosec == 0
        ):
            self._warn("Skipping positions with an invalid image timestamp")
            return
        if any(
            not all(
                isfinite(value)
                for value in (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                )
            )
            for pose in message.poses
        ):
            self._warn("Skipping positions with non-finite camera XYZ")
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                message.header.frame_id,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as error:
            self._warn(
                f"Skipping positions because exact-time TF failed: {error}"
            )
            return

        try:
            mapped = transform_positions(
                message, transform, self._target_frame
            )
        except Exception as error:  # Malformed TF must not stop perception.
            self._warn(
                f"Skipping positions because TF output is invalid: {error}"
            )
            return
        self._publisher.publish(mapped)

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)


def main(args=None):
    """Run until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorMapTransformNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
