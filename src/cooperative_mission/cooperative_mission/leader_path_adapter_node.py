"""Convert a Leader Nav2 axle path into an object path for formation preview."""

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
    leader_path_to_object_path,
)
from .motion import Pose2D
from .ros_common import yaw_from_quaternion


class LeaderPathAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("cooperative_leader_path_adapter")
        self.declare_parameter("leader_path_topic", "/plan")
        self.declare_parameter("object_path_topic", "/cooperation/object_path")
        self.declare_parameter(
            "status_topic", "/cooperation/leader_path_adapter/status"
        )
        self.declare_parameter("axle_to_hinge", 0.125)
        self.declare_parameter("hinge_to_contact", 0.1325)
        self.declare_parameter("object_center_to_contact", 0.0525)
        self.declare_parameter("hinge_limit_deg", 15.0)
        self.declare_parameter("curvature_margin", 0.8)
        self.declare_parameter("lateral_tolerance", 0.005)

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
        self._margin = float(self.get_parameter("curvature_margin").value)
        self._lateral_tolerance = float(
            self.get_parameter("lateral_tolerance").value
        )
        self._object_pub = self.create_publisher(
            Path, str(self.get_parameter("object_path_topic").value), 1
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 1
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("leader_path_topic").value),
            self._on_leader_path,
            1,
        )
        self.get_logger().info(
            "Waiting for Leader axle path on %s"
            % self.get_parameter("leader_path_topic").value
        )

    def _on_leader_path(self, message: Path) -> None:
        if not message.header.frame_id:
            self._publish_status("PLAN_REJECTED reason=leader_path_frame_empty")
            return
        if len(message.poses) < 3:
            self._publish_status(
                "PLAN_REJECTED reason=leader_path_needs_at_least_3_poses"
            )
            return
        leader_poses = []
        for stamped in message.poses:
            pose = stamped.pose
            leader_poses.append(
                Pose2D(
                    pose.position.x,
                    pose.position.y,
                    yaw_from_quaternion(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
            )
        try:
            object_poses, formation = leader_path_to_object_path(
                leader_poses,
                self._geometry,
                curvature_margin=self._margin,
                lateral_tolerance=self._lateral_tolerance,
            )
            leader_direction = drive_direction(formation.leader)
            follower_direction = drive_direction(formation.follower)
        except ValueError as error:
            self._publish_status("PLAN_REJECTED reason=%s" % error)
            return

        output = Path()
        output.header = message.header
        for pose2d in object_poses:
            stamped = PoseStamped()
            stamped.header = output.header
            stamped.pose.position.x = pose2d.x
            stamped.pose.position.y = pose2d.y
            stamped.pose.orientation.z = math.sin(0.5 * pose2d.yaw)
            stamped.pose.orientation.w = math.cos(0.5 * pose2d.yaw)
            output.poses.append(stamped)
        self._object_pub.publish(output)
        self._publish_status(
            "KINEMATIC_LEADER_PATH_CONVERTED poses=%d frame=%s "
            "max_curvature=%.4f 1/m max_hinge=%.2f deg "
            "leader_drive=%s follower_drive=%s execution=PREVIEW_ONLY"
            % (
                len(object_poses),
                message.header.frame_id,
                formation.max_abs_curvature,
                math.degrees(
                    max(map(abs, formation.leader_hinge_angles), default=0.0)
                ),
                leader_direction,
                follower_direction,
            )
        )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LeaderPathAdapterNode()
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
