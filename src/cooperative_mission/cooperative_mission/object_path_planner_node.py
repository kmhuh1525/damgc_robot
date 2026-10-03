"""Plan a collision-checked cooperative object path between two planar poses."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .hinged_formation import HingeGeometry
from .motion import Pose2D
from .object_path_planner import plan_object_path
from .ros_common import yaw_from_quaternion


class CooperativeObjectPathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("cooperative_object_path_planner")
        self.declare_parameter("start_topic", "/cooperation/object_start")
        self.declare_parameter("goal_topic", "/cooperation/object_goal")
        self.declare_parameter("object_path_topic", "/cooperation/object_path")
        self.declare_parameter(
            "status_topic", "/cooperation/object_path_planner/status"
        )
        self.declare_parameter("axle_to_hinge", 0.125)
        self.declare_parameter("hinge_to_contact", 0.1325)
        self.declare_parameter("object_center_to_contact", 0.0525)
        self.declare_parameter("hinge_limit_deg", 15.0)
        self.declare_parameter("curvature_margin", 0.8)
        self.declare_parameter("lateral_tolerance", 0.005)
        self.declare_parameter("sample_step", 0.05)
        self.declare_parameter("obstacle_circles", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("workspace_bounds", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("robot_radius", 0.32)
        self.declare_parameter("safety_margin", 0.05)

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
        self._sample_step = float(self.get_parameter("sample_step").value)
        raw_obstacles = list(self.get_parameter("obstacle_circles").value)
        if len(raw_obstacles) % 3:
            raise ValueError("obstacle_circles must contain x, y, radius triples")
        self._obstacles = tuple(
            tuple(float(value) for value in raw_obstacles[index:index + 3])
            for index in range(0, len(raw_obstacles), 3)
        )
        raw_bounds = list(self.get_parameter("workspace_bounds").value)
        if raw_bounds and len(raw_bounds) != 4:
            raise ValueError("workspace_bounds must be [xmin, xmax, ymin, ymax]")
        self._bounds = tuple(float(value) for value in raw_bounds) if raw_bounds else None
        self._robot_radius = float(self.get_parameter("robot_radius").value)
        self._safety_margin = float(self.get_parameter("safety_margin").value)
        self._start = None
        self._goal = None
        self._start_frame = ""
        self._goal_frame = ""

        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("object_path_topic").value), 1
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 1
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 1
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("start_topic").value),
            self._on_start,
            1,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal,
            1,
        )
        self.get_logger().info("Cooperative object pose planner ready")
        self._publish_environment()

    def _on_start(self, message: PoseStamped) -> None:
        self._start = self._to_pose(message)
        self._start_frame = message.header.frame_id
        self._publish_environment()
        self._try_plan()

    def _on_goal(self, message: PoseStamped) -> None:
        self._goal = self._to_pose(message)
        self._goal_frame = message.header.frame_id
        self._try_plan()

    @staticmethod
    def _to_pose(message: PoseStamped) -> Pose2D:
        p = message.pose
        q = p.orientation
        return Pose2D(
            p.position.x,
            p.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def _try_plan(self) -> None:
        if self._start is None or self._goal is None:
            return
        if not self._start_frame or self._start_frame != self._goal_frame:
            self._publish_status(
                "PLAN_REJECTED frame_mismatch start=%s goal=%s"
                % (self._start_frame, self._goal_frame)
            )
            return
        try:
            object_poses, formation = plan_object_path(
                self._start,
                self._goal,
                self._geometry,
                step=self._sample_step,
                curvature_margin=self._margin,
                lateral_tolerance=self._lateral_tolerance,
                obstacles=self._obstacles,
                workspace_bounds=self._bounds,
                robot_radius=self._robot_radius,
                safety_margin=self._safety_margin,
            )
        except ValueError as error:
            self._publish_status("PLAN_REJECTED reason=%s" % error)
            return

        path = Path()
        path.header.frame_id = self._start_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for pose2d in object_poses:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose.position.x = pose2d.x
            stamped.pose.position.y = pose2d.y
            stamped.pose.orientation.z = math.sin(0.5 * pose2d.yaw)
            stamped.pose.orientation.w = math.cos(0.5 * pose2d.yaw)
            path.poses.append(stamped)
        self._path_pub.publish(path)
        length = sum(
            math.hypot(b.x - a.x, b.y - a.y)
            for a, b in zip(object_poses, object_poses[1:])
        )
        max_hinge = max(
            (abs(value) for value in formation.leader_hinge_angles), default=0.0
        )
        self._publish_status(
            "KINEMATIC_OBJECT_PATH_PLANNED poses=%d length=%.2f m "
            "max_curvature=%.3f 1/m max_hinge=%.2f deg max_lateral_ratio=%.4f "
            "obstacles=%d model=quasi_static_hinge"
            % (
                len(object_poses),
                length,
                formation.max_abs_curvature,
                math.degrees(max_hinge),
                formation.max_lateral_ratio,
                len(self._obstacles),
            )
        )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)

    def _publish_environment(self) -> None:
        markers = MarkerArray()
        frame_id = self._start_frame or "odom"
        now = self.get_clock().now().to_msg()
        for index, (x, y, radius) in enumerate(self._obstacles):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = "obstacles"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.01
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2.0 * radius
            marker.scale.z = 0.02
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.15, 0.1, 0.8
            markers.markers.append(marker)
        if self._bounds is not None:
            xmin, xmax, ymin, ymax = self._bounds
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = "workspace"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.85, 0.85, 0.85, 0.9
            for x, y in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)):
                point = Point()
                point.x, point.y, point.z = x, y, 0.025
                marker.points.append(point)
            markers.markers.append(marker)
        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CooperativeObjectPathPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
