"""Publish repeatable object start/goal poses for the offline planner demo."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CooperativeDemoPlanNode(Node):
    """Feed a 90-degree object transport task to the cooperative planner."""

    def __init__(self) -> None:
        super().__init__("cooperative_demo_task")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("start_topic", "/cooperation/object_start")
        self.declare_parameter("goal_topic", "/cooperation/object_goal")
        self.declare_parameter("start_x", -0.8)
        self.declare_parameter("start_y", -1.5)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("goal_x", 0.7)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", math.pi / 2.0)
        self.declare_parameter("publish_period", 1.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        period = float(self.get_parameter("publish_period").value)
        if not self._frame_id or not math.isfinite(period) or period <= 0.0:
            raise ValueError("frame_id and positive publish_period are required")
        self._start_values = tuple(
            float(self.get_parameter(name).value)
            for name in ("start_x", "start_y", "start_yaw")
        )
        self._goal_values = tuple(
            float(self.get_parameter(name).value)
            for name in ("goal_x", "goal_y", "goal_yaw")
        )
        if not all(math.isfinite(value) for value in self._start_values + self._goal_values):
            raise ValueError("start/goal pose parameters must be finite")

        self._start_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("start_topic").value), 1
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 1
        )
        self._timer = self.create_timer(period, self._publish_task)
        self.get_logger().warning(
            "Publishing synthetic object start/goal on cooperative-only topics"
        )

    def _publish_task(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._start_pub.publish(self._to_pose(self._start_values, stamp))
        self._goal_pub.publish(self._to_pose(self._goal_values, stamp))

    def _to_pose(self, values, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, yaw = values
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CooperativeDemoPlanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
