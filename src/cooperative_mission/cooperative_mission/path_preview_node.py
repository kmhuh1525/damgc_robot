"""Generate both robot axle paths from a cooperative object-center path."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .hinged_formation import (
    HingeGeometry,
    drive_direction,
    object_path_to_robot_paths,
)
from .motion import Pose2D
from .ros_common import yaw_from_quaternion


class CooperativePathPreviewNode(Node):
    """Apply bounded passive-hinge kinematics to an object path."""

    def __init__(self) -> None:
        super().__init__("cooperative_path_preview")
        self.declare_parameter("geometry_ready", False)
        self.declare_parameter("object_path_topic", "/cooperation/object_path")
        self.declare_parameter("leader_path_topic", "/cooperation/leader_path_preview")
        self.declare_parameter("follower_path_topic", "/cooperation/follower_path_preview")
        self.declare_parameter("status_topic", "/cooperation/path_preview/status")
        self.declare_parameter("axle_to_hinge", 0.125)
        self.declare_parameter("hinge_to_contact", 0.1325)
        self.declare_parameter("object_center_to_contact", 0.0525)
        self.declare_parameter("hinge_limit_deg", 15.0)
        self.declare_parameter("curvature_margin", 0.8)
        self.declare_parameter("lateral_tolerance", 0.01)

        self._geometry_ready = bool(self.get_parameter("geometry_ready").value)
        self._geometry = HingeGeometry(
            axle_to_hinge=float(self.get_parameter("axle_to_hinge").value),
            hinge_to_contact=float(self.get_parameter("hinge_to_contact").value),
            object_center_to_contact=float(
                self.get_parameter("object_center_to_contact").value
            ),
            hinge_limit=math.radians(
                float(self.get_parameter("hinge_limit_deg").value)
            ),
        )
        self._geometry.validate()
        self._curvature_margin = float(self.get_parameter("curvature_margin").value)
        if not 0.0 < self._curvature_margin <= 1.0:
            raise ValueError("curvature_margin must be in (0, 1]")
        self._lateral_tolerance = float(self.get_parameter("lateral_tolerance").value)
        if not math.isfinite(self._lateral_tolerance) or self._lateral_tolerance < 0.0:
            raise ValueError("lateral_tolerance must be finite and non-negative")

        self._leader_pub = self.create_publisher(
            Path, str(self.get_parameter("leader_path_topic").value), 1
        )
        self._follower_pub = self.create_publisher(
            Path, str(self.get_parameter("follower_path_topic").value), 1
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 1
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("object_path_topic").value),
            self._on_object_path,
            1,
        )
        self.get_logger().info(
            "Hinged cooperative path planner ready; geometry=%s; outputs are preview only"
            % ("configured" if self._geometry_ready else "not configured")
        )
        if not self._geometry_ready:
            self._publish_status("HINGE_GEOMETRY_NOT_CONFIGURED")

    def _on_object_path(self, message: Path) -> None:
        if not self._geometry_ready:
            self._publish_status("HINGE_GEOMETRY_NOT_CONFIGURED")
            return
        if not message.poses:
            self._publish_status("EMPTY_OBJECT_PATH")
            return
        if not message.header.frame_id:
            self._publish_status("OBJECT_PATH_FRAME_EMPTY")
            return

        object_poses = []
        for stamped in message.poses:
            p = stamped.pose
            object_poses.append(
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
        try:
            formation = object_path_to_robot_paths(
                object_poses,
                self._geometry,
                curvature_margin=self._curvature_margin,
                lateral_tolerance=self._lateral_tolerance,
            )
        except ValueError as error:
            self._publish_status("OBJECT_PATH_REJECTED reason=%s" % error)
            return

        leader = self._to_path(message, formation.leader)
        follower = self._to_path(message, formation.follower)
        self._leader_pub.publish(leader)
        self._follower_pub.publish(follower)
        max_hinge = max(
            (abs(value) for value in formation.leader_hinge_angles), default=0.0
        )
        leader_direction = drive_direction(formation.leader)
        follower_direction = drive_direction(formation.follower)
        status = (
            "KINEMATIC_PATH_CANDIDATE poses=%d frame=%s model=quasi_static_hinge "
            "max_curvature=%.4f 1/m "
            "curvature_limit=%.4f 1/m max_hinge=%.2f deg hinge_limit=%.2f deg "
            "max_lateral_ratio=%.5f leader_drive=%s follower_drive=%s "
            "execution=PREVIEW_ONLY"
        ) % (
            len(formation.leader),
            message.header.frame_id,
            formation.max_abs_curvature,
            self._geometry.curvature_limit(self._curvature_margin),
            math.degrees(max_hinge),
            math.degrees(self._geometry.hinge_limit * self._curvature_margin),
            formation.max_lateral_ratio,
            leader_direction,
            follower_direction,
        )
        self._publish_status(status)

    @staticmethod
    def _to_path(source: Path, poses) -> Path:
        path = Path()
        path.header = source.header
        for pose2d in poses:
            stamped = PoseStamped()
            stamped.header = source.header
            stamped.pose.position.x = pose2d.x
            stamped.pose.position.y = pose2d.y
            stamped.pose.position.z = 0.0
            stamped.pose.orientation.z = math.sin(0.5 * pose2d.yaw)
            stamped.pose.orientation.w = math.cos(0.5 * pose2d.yaw)
            path.poses.append(stamped)
        return path

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)


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
