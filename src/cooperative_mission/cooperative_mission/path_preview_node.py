"""Preview the Follower formation path derived from the Leader Nav2 plan.

This node is diagnostic only. It publishes no velocity command and cannot
drive either robot. The transformed path remains in the Leader plan frame.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .formation import GraspGeometry, leader_pose_to_follower, leader_twist_to_follower
from .motion import Pose2D
from .ros_common import yaw_from_quaternion


class CooperativePathPreviewNode(Node):
    """Transform /plan through the calibrated rigid-grasp geometry."""

    def __init__(self) -> None:
        super().__init__("cooperative_path_preview")
        self._declare_parameters()
        self._geometry_ready = bool(self.get_parameter("geometry_ready").value)
        self._geometry: Optional[GraspGeometry] = None
        if self._geometry_ready:
            self._geometry = self._load_geometry()

        self._leader_pose: Optional[Pose2D] = None
        self._leader_pose_receipt: Optional[float] = None
        self._leader_cmd: Optional[Twist] = None
        self._leader_cmd_receipt: Optional[float] = None
        self._lateral_tolerance = float(
            self.get_parameter("lateral_velocity_tolerance").value
        )
        self._pose_timeout = float(self.get_parameter("pose_timeout").value)
        self._twist_timeout = float(self.get_parameter("twist_timeout").value)
        if (self._lateral_tolerance < 0.0 or self._pose_timeout <= 0.0
                or self._twist_timeout <= 0.0):
            raise ValueError("tolerances must be non-negative and pose_timeout positive")

        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("follower_path_topic").value), 1
        )
        self._twist_pub = self.create_publisher(
            Twist, str(self.get_parameter("follower_twist_topic").value), 1
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 1
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("leader_path_topic").value),
            self._on_path,
            1,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("leader_odom_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("leader_twist_topic").value),
            self._on_leader_twist,
            1,
        )
        self.get_logger().info(
            "Cooperative path preview ready; geometry=%s; output is diagnostic only"
            % ("configured" if self._geometry_ready else "not configured")
        )
        if not self._geometry_ready:
            self._publish_status("GEOMETRY_NOT_CONFIGURED")

    def _declare_parameters(self) -> None:
        d = self.declare_parameter
        d("geometry_ready", False)
        d("leader_odom_frame", "odom")
        d("box_length", 0.0)
        d("box_width", 0.0)
        for prefix in (
            "leader_base_to_gripper",
            "object_to_leader_gripper",
            "object_to_follower_gripper",
            "follower_base_to_gripper",
        ):
            d(prefix + ".x", 0.0)
            d(prefix + ".y", 0.0)
            d(prefix + ".yaw", 0.0)
        d("lateral_velocity_tolerance", 0.01)
        d("pose_timeout", 0.3)
        d("twist_timeout", 0.5)
        d("leader_path_topic", "/plan")
        d("leader_odom_topic", "/visual_slam/tracking/odometry")
        d("leader_twist_topic", "/nav2/cmd_vel")
        d("follower_path_topic", "/cooperation/follower_path_preview")
        d("follower_twist_topic", "/cooperation/follower_twist_preview")
        d("status_topic", "/cooperation/path_preview/status")

    def _load_geometry(self) -> GraspGeometry:
        def pose(prefix: str) -> Pose2D:
            return Pose2D(
                float(self.get_parameter(prefix + ".x").value),
                float(self.get_parameter(prefix + ".y").value),
                float(self.get_parameter(prefix + ".yaw").value),
            )

        geometry = GraspGeometry(
            box_length=float(self.get_parameter("box_length").value),
            box_width=float(self.get_parameter("box_width").value),
            leader_base_to_gripper=pose("leader_base_to_gripper"),
            object_to_leader_gripper=pose("object_to_leader_gripper"),
            object_to_follower_gripper=pose("object_to_follower_gripper"),
            follower_base_to_gripper=pose("follower_base_to_gripper"),
        )
        geometry.validate()
        return geometry

    def _on_path(self, message: Path) -> None:
        if not self._geometry_ready or self._geometry is None:
            self._publish_status("GEOMETRY_NOT_CONFIGURED")
            return
        expected_frame = str(self.get_parameter("leader_odom_frame").value)
        if message.header.frame_id != expected_frame:
            self._publish_status(
                "FRAME_MISMATCH expected=%s received=%s"
                % (expected_frame, message.header.frame_id)
            )
            return
        if not message.poses:
            self._publish_status("EMPTY_PLAN")
            return

        transformed = Path()
        transformed.header = message.header
        for stamped_pose in message.poses:
            p = stamped_pose.pose
            leader_pose = Pose2D(
                p.position.x,
                p.position.y,
                yaw_from_quaternion(
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    p.orientation.w,
                ),
            )
            follower_pose = leader_pose_to_follower(leader_pose, self._geometry)
            output = PoseStamped()
            output.header = stamped_pose.header
            output.header.frame_id = expected_frame
            output.pose.position.x = follower_pose.x
            output.pose.position.y = follower_pose.y
            output.pose.position.z = p.position.z
            half_yaw = 0.5 * follower_pose.yaw
            output.pose.orientation.z = math.sin(half_yaw)
            output.pose.orientation.w = math.cos(half_yaw)
            transformed.poses.append(output)
        self._path_pub.publish(transformed)

        status = "PATH_PREVIEW_READY poses=%d frame=%s" % (
            len(transformed.poses), expected_frame
        )
        diagnostic = self._current_twist_diagnostic()
        if diagnostic:
            status += " " + diagnostic
        self._publish_status(status)

    def _on_odometry(self, message: Odometry) -> None:
        expected_frame = str(self.get_parameter("leader_odom_frame").value)
        if message.header.frame_id != expected_frame:
            self._publish_status(
                "ODOMETRY_FRAME_MISMATCH expected=%s received=%s"
                % (expected_frame, message.header.frame_id)
            )
            self._leader_pose = None
            self._leader_pose_receipt = None
            return
        q = message.pose.pose.orientation
        self._leader_pose = Pose2D(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self._leader_pose_receipt = time.monotonic()
        if self._leader_cmd is not None:
            self._publish_twist_preview()

    def _on_leader_twist(self, message: Twist) -> None:
        self._leader_cmd = message
        self._leader_cmd_receipt = time.monotonic()
        if self._leader_pose is not None:
            self._publish_twist_preview()

    def _publish_twist_preview(self) -> None:
        if self._geometry is None or self._leader_pose is None or self._leader_cmd is None:
            return
        if self._leader_pose_receipt is None or self._leader_cmd_receipt is None:
            return
        now = time.monotonic()
        if now - self._leader_pose_receipt > self._pose_timeout:
            self._publish_status("STALE_LEADER_ODOMETRY")
            return
        if now - self._leader_cmd_receipt > self._twist_timeout:
            self._publish_status("STALE_LEADER_TWIST")
            return
        preview = leader_twist_to_follower(
            self._leader_pose,
            self._leader_cmd.linear.x,
            self._leader_cmd.angular.z,
            self._geometry,
        )
        output = Twist()
        output.linear.x = preview.linear_x
        output.angular.z = preview.angular_z
        self._twist_pub.publish(output)
        state = "TWIST_PREVIEW_FEASIBLE" if preview.is_differential_drive_feasible(
            self._lateral_tolerance
        ) else "TWIST_PREVIEW_INFEASIBLE"
        self._publish_status(
            "%s follower_vx=%.4f required_vy=%.4f omega=%.4f"
            % (state, preview.linear_x, preview.linear_y_required, preview.angular_z)
        )

    def _current_twist_diagnostic(self) -> str:
        if self._leader_pose is None or self._leader_cmd is None:
            return "WAITING_ODOMETRY_OR_TWIST"
        if self._leader_pose_receipt is None or time.monotonic() - self._leader_pose_receipt > self._pose_timeout:
            return "STALE_LEADER_ODOMETRY"
        if self._leader_cmd_receipt is None or time.monotonic() - self._leader_cmd_receipt > self._twist_timeout:
            return "STALE_LEADER_TWIST"
        preview = leader_twist_to_follower(
            self._leader_pose,
            self._leader_cmd.linear.x,
            self._leader_cmd.angular.z,
            self._geometry,
        )
        state = "KINEMATICALLY_FEASIBLE" if preview.is_differential_drive_feasible(
            self._lateral_tolerance
        ) else "KINEMATICALLY_INFEASIBLE"
        return "%s required_lateral=%.4f" % (state, preview.linear_y_required)

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CooperativePathPreviewNode()
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
