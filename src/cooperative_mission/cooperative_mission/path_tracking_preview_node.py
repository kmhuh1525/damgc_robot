"""Publish preview-only pure-pursuit commands for the Follower path."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .motion import Pose2D
from .path_tracking import compute_tracking_command
from .ros_common import yaw_from_quaternion


class CooperativePathTrackingPreviewNode(Node):
    """Track a Follower axle path mathematically without publishing motor input."""

    def __init__(self) -> None:
        super().__init__("cooperative_path_tracking_preview")
        self.declare_parameter("path_topic", "/cooperation/follower_path_preview")
        self.declare_parameter("odom_topic", "/follower/odom/raw")
        self.declare_parameter(
            "command_preview_topic", "/cooperation/follower/path_tracking/cmd_vel_preview"
        )
        self.declare_parameter(
            "status_topic", "/cooperation/follower/path_tracking/status"
        )
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("lookahead_distance", 0.20)
        self.declare_parameter("max_path_error", 0.40)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.20)
        self.declare_parameter("lateral_acceleration", 0.08)
        self.declare_parameter("odom_timeout", 0.50)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path = ()
        self._path_frame = ""
        self._progress = 0
        self._robot_pose = None
        self._odom_frame = ""
        self._odom_received_at = 0.0
        self._last_status_at = 0.0
        self._last_status_key = ""
        self._command_pub = self.create_publisher(
            Twist, str(self.get_parameter("command_preview_topic").value), 1
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 1
        )
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self._on_path, 1
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odometry, 10
        )
        rate = float(self.get_parameter("publish_rate").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate must be finite and positive")
        self.create_timer(1.0 / rate, self._update)
        self.get_logger().info(
            "Pure-pursuit preview ready; output topic is isolated from motor commands"
        )

    def _on_path(self, message: Path) -> None:
        if not message.header.frame_id or len(message.poses) < 2:
            self._path = ()
            self._publish_status("WAITING_FOR_VALID_FOLLOWER_PATH")
            return
        poses = []
        for stamped in message.poses:
            p = stamped.pose
            poses.append(
                Pose2D(
                    p.position.x,
                    p.position.y,
                    yaw_from_quaternion(
                        p.orientation.x,
                        p.orientation.y,
                        p.orientation.z,
                        p.orientation.w,
                    ),
                )
            )
        if self._path and self._path_frame == message.header.frame_id:
            self._progress = min(self._progress, len(poses) - 1)
        else:
            self._progress = 0
        self._path = tuple(poses)
        self._path_frame = message.header.frame_id

    def _on_odometry(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        self._robot_pose = Pose2D(
            p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)
        )
        self._odom_frame = message.header.frame_id
        self._odom_received_at = time.monotonic()

    def _pose_in_path_frame(self) -> Pose2D:
        if not self._path_frame or not self._odom_frame:
            raise TransformException("path or odometry frame is empty")
        assert self._robot_pose is not None
        if self._path_frame == self._odom_frame:
            return self._robot_pose
        transform = self._tf_buffer.lookup_transform(
            self._path_frame, self._odom_frame, Time()
        ).transform
        tq = transform.rotation
        transform_yaw = yaw_from_quaternion(tq.x, tq.y, tq.z, tq.w)
        c, s = math.cos(transform_yaw), math.sin(transform_yaw)
        return Pose2D(
            transform.translation.x + c * self._robot_pose.x - s * self._robot_pose.y,
            transform.translation.y + s * self._robot_pose.x + c * self._robot_pose.y,
            math.atan2(
                math.sin(transform_yaw + self._robot_pose.yaw),
                math.cos(transform_yaw + self._robot_pose.yaw),
            ),
        )

    def _update(self) -> None:
        command = Twist()
        if len(self._path) < 2:
            self._publish_command(command)
            return
        timeout = float(self.get_parameter("odom_timeout").value)
        if (
            self._robot_pose is None
            or time.monotonic() - self._odom_received_at > timeout
        ):
            self._publish_command(command)
            self._publish_status("WAITING_FOR_FRESH_FOLLOWER_ODOMETRY")
            return
        try:
            robot = self._pose_in_path_frame()
            result = compute_tracking_command(
                self._path,
                robot,
                self._progress,
                lookahead_distance=float(self.get_parameter("lookahead_distance").value),
                max_path_error=float(self.get_parameter("max_path_error").value),
                goal_tolerance=float(self.get_parameter("goal_tolerance").value),
                max_linear_speed=float(self.get_parameter("max_linear_speed").value),
                max_angular_speed=float(self.get_parameter("max_angular_speed").value),
                lateral_acceleration=float(self.get_parameter("lateral_acceleration").value),
            )
        except (TransformException, ValueError) as error:
            self._publish_command(command)
            self._publish_status("TRACKING_REJECTED reason=%s" % error)
            return
        self._progress = result.progress_index
        command.linear.x = result.command.linear_x
        command.angular.z = result.command.angular_z
        self._publish_command(command)
        self._publish_status(
            "PATH_TRACKING_PREVIEW state=%s direction=%s v=%.3f m/s w=%.3f rad/s "
            "cross_track=%.3f m progress=%d/%d"
            % (
                result.detail.upper(),
                result.direction,
                result.command.linear_x,
                result.command.angular_z,
                result.cross_track_error,
                result.progress_index + 1,
                len(self._path),
            )
        )

    def _publish_command(self, command: Twist) -> None:
        self._command_pub.publish(command)

    def _publish_status(self, text: str) -> None:
        now = time.monotonic()
        key = text.split(" ", 1)[0]
        if key == self._last_status_key and now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        self._last_status_key = key
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CooperativePathTrackingPreviewNode()
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
