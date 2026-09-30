"""ROS 2 wrapper around :class:`FollowerMission` (runs on the Follower Orin).

Owns ``/follower/mission/cmd_vel`` which the mission launch remaps into the
``COOPERATION`` input of ``follower_command_selector``, switches the selector
source through its ``source_mode`` parameter, enables the existing approach
controller / velocity guard and drives the Follower Dynamixel gripper.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from std_srvs.srv import SetBool

from .actions import Action, ActionKind
from .follower_logic import FollowerMission, FollowerMissionConfig, config_segments
from .motion import GripperParameters, ManeuverParameters, SearchParameters, ZERO
from .protocol import FollowerState, decode_command, encode_status
from .ros_common import (
    COMMAND_QOS,
    LINK_QOS,
    SENSOR_QOS,
    call_set_bool,
    to_twist,
    yaw_from_quaternion,
)


class FollowerMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor")
        self._declare()
        config = self._load_config()
        self._rate = float(self.get_parameter("update_rate").value)
        if not math.isfinite(self._rate) or self._rate <= 0.0:
            raise ValueError("update_rate must be positive")
        self._require_gripper = bool(self.get_parameter("require_gripper").value)
        self._needs_odom = bool(config.reposition_segments)
        self._odom_timeout = config.odom_timeout
        self._last_odom_time: Optional[float] = None
        self._logic = FollowerMission(config)
        p = lambda name: str(self.get_parameter(name).value)  # noqa: E731

        self._coop_topic = p("cooperation_command_topic")
        self._gripper_topic = p("gripper_command_topic")
        self._coop_pub = self.create_publisher(Twist, self._coop_topic, COMMAND_QOS)
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, p("gripper_command_topic"), 10
        )
        self._status_pub = self.create_publisher(String, p("status_topic"), LINK_QOS)

        self.create_subscription(Bool, p("detected_topic"), self._on_detected, SENSOR_QOS)
        self.create_subscription(Int32, p("tag_id_topic"), self._on_tag_id, SENSOR_QOS)
        self.create_subscription(String, p("alignment_state_topic"), self._on_alignment, SENSOR_QOS)
        self.create_subscription(Odometry, p("odom_topic"), self._on_odom, SENSOR_QOS)
        self.create_subscription(String, p("gripper_status_topic"), self._on_gripper_status, 10)
        self.create_subscription(String, p("leader_command_topic"), self._on_leader_command, LINK_QOS)
        # /mission/state is TRANSIENT_LOCAL on the Leader; a VOLATILE reader is compatible.
        self.create_subscription(String, p("leader_state_topic"), self._on_leader_state, LINK_QOS)
        self.create_subscription(Twist, p("target_velocity_topic"), self._on_target, COMMAND_QOS)

        self._approach_client = self.create_client(SetBool, p("approach_enable_service"))
        self._guard_client = self.create_client(SetBool, p("guard_enable_service"))
        self._selector_client = self.create_client(SetParameters, p("selector_parameter_service"))

        self._last_state: Optional[FollowerState] = None
        self.create_timer(1.0 / self._rate, self._on_timer)
        self.create_timer(1.0 / float(self.get_parameter("status_rate").value), self._publish_status)
        self.create_timer(0.5, self._update_readiness)
        self.get_logger().info(
            "Follower mission executor ready: leader_facing_opposite=%s, reposition=%s"
            % (config.leader_facing_opposite, list(config.reposition_segments) or "none")
        )

    # ---------------------------------------------------------- parameters
    def _declare(self) -> None:
        d = self.declare_parameter
        d("update_rate", 50.0)
        d("status_rate", 10.0)
        d("require_gripper", True)
        d("detected_topic", "/follower/supply/detected")
        d("tag_id_topic", "/follower/supply/tag_id")
        d("alignment_state_topic", "/follower/base_alignment/state")
        d("odom_topic", "/follower/odom/raw")
        d("gripper_command_topic", "/follower/dynamixel/command")
        d("gripper_status_topic", "/follower/dynamixel/status")
        d("cooperation_command_topic", "/follower/mission/cmd_vel")
        d("approach_enable_service", "/follower/approach/enable")
        d("guard_enable_service", "/follower/velocity_guard/enable")
        d("selector_parameter_service", "/follower/command_selector/set_parameters")
        d("leader_command_topic", "/mission/follower_command")
        d("leader_state_topic", "/mission/state")
        d("target_velocity_topic", "/cooperation/target_velocity")
        d("status_topic", "/follower/mission/status")

        defaults = FollowerMissionConfig()
        d("target_tag_id", defaults.target_tag_id)
        d("tag_acquire_time", defaults.tag_acquire_time)
        d("search_angular_speed", defaults.search.angular_speed)
        d("search_direction", defaults.search.direction)
        d("search_step_angle_deg", math.degrees(defaults.search.step_angle))
        d("search_dwell_time", defaults.search.dwell_time)
        d("search_max_angle_deg", math.degrees(defaults.search.max_angle))
        # ROS 2 cannot infer the type of an empty list; [""] means "no segments".
        d("reposition_segments", [""])
        m = defaults.maneuver
        d("maneuver_drive_speed", m.drive_speed)
        d("maneuver_min_drive_speed", m.min_drive_speed)
        d("maneuver_turn_speed", m.turn_speed)
        d("maneuver_min_turn_speed", m.min_turn_speed)
        d("maneuver_drive_tolerance", m.drive_tolerance)
        d("maneuver_turn_tolerance_deg", math.degrees(m.turn_tolerance))
        d("odom_timeout", defaults.odom_timeout)
        d("reacquire_timeout", defaults.reacquire_timeout)
        d("max_reacquire", defaults.max_reacquire)
        d("approach_timeout", defaults.approach_timeout)
        d("aligned_hold_time", defaults.aligned_hold_time)
        g = defaults.gripper
        d("gripper_open_raw", g.open_raw)
        d("gripper_close_raw", g.close_raw)
        d("lift_raw", g.lift_raw)
        d("lower_raw", g.lower_raw)
        d("approach_rx64_raw", g.approach_rx64_raw)
        d("rx64_min", g.rx64_min)
        d("rx64_max", g.rx64_max)
        d("grasp_settle_time", defaults.grasp_settle_time)
        d("lift_duration", defaults.lift_duration)
        d("release_lower_time", defaults.release_lower_time)
        d("release_open_settle", defaults.release_open_settle)
        d("leader_timeout", defaults.leader_timeout)
        d("target_velocity_timeout", defaults.target_velocity_timeout)
        d("leader_facing_opposite", defaults.leader_facing_opposite)
        d("max_transport_linear", defaults.max_transport_linear)
        d("max_transport_angular", defaults.max_transport_angular)
        d("action_timeout", defaults.action_timeout)

    def _load_config(self) -> FollowerMissionConfig:
        v = lambda name: self.get_parameter(name).value  # noqa: E731
        return FollowerMissionConfig(
            target_tag_id=int(v("target_tag_id")),
            tag_acquire_time=float(v("tag_acquire_time")),
            search=SearchParameters(
                angular_speed=float(v("search_angular_speed")),
                direction=1.0 if float(v("search_direction")) >= 0.0 else -1.0,
                step_angle=math.radians(float(v("search_step_angle_deg"))),
                dwell_time=float(v("search_dwell_time")),
                max_angle=math.radians(float(v("search_max_angle_deg"))),
            ),
            reposition_segments=config_segments(v("reposition_segments") or []),
            maneuver=ManeuverParameters(
                drive_speed=float(v("maneuver_drive_speed")),
                min_drive_speed=float(v("maneuver_min_drive_speed")),
                turn_speed=float(v("maneuver_turn_speed")),
                min_turn_speed=float(v("maneuver_min_turn_speed")),
                drive_tolerance=float(v("maneuver_drive_tolerance")),
                turn_tolerance=math.radians(float(v("maneuver_turn_tolerance_deg"))),
            ),
            odom_timeout=float(v("odom_timeout")),
            reacquire_timeout=float(v("reacquire_timeout")),
            max_reacquire=int(v("max_reacquire")),
            approach_timeout=float(v("approach_timeout")),
            aligned_hold_time=float(v("aligned_hold_time")),
            gripper=GripperParameters(
                open_raw=int(v("gripper_open_raw")),
                close_raw=int(v("gripper_close_raw")),
                lift_raw=int(v("lift_raw")),
                lower_raw=int(v("lower_raw")),
                approach_rx64_raw=int(v("approach_rx64_raw")),
                rx64_min=int(v("rx64_min")),
                rx64_max=int(v("rx64_max")),
            ),
            grasp_settle_time=float(v("grasp_settle_time")),
            lift_duration=float(v("lift_duration")),
            release_lower_time=float(v("release_lower_time")),
            release_open_settle=float(v("release_open_settle")),
            leader_timeout=float(v("leader_timeout")),
            target_velocity_timeout=float(v("target_velocity_timeout")),
            leader_facing_opposite=bool(v("leader_facing_opposite")),
            max_transport_linear=float(v("max_transport_linear")),
            max_transport_angular=float(v("max_transport_angular")),
            action_timeout=float(v("action_timeout")),
        )

    # ------------------------------------------------------------- inputs
    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_detected(self, message: Bool) -> None:
        self._logic.on_tag_detected(bool(message.data), self._now())

    def _on_tag_id(self, message: Int32) -> None:
        self._logic.on_tag_id(int(message.data), self._now())

    def _on_alignment(self, message: String) -> None:
        self._logic.on_alignment_state(message.data, self._now())

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        now = self._now()
        self._last_odom_time = now
        self._logic.on_odometry(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
            now,
        )

    def _on_gripper_status(self, message: String) -> None:
        if message.data.startswith("ERROR"):
            self.get_logger().error(f"Follower Dynamixel: {message.data}")
        self._logic.on_gripper_status(message.data, self._now())

    def _on_leader_command(self, message: String) -> None:
        command = decode_command(message.data)
        if command is None:
            self.get_logger().warning("ignored malformed Leader command", throttle_duration_sec=2.0)
            return
        self._logic.on_command(command, self._now())
        # Execute the resulting actions (e.g. the lift) before acknowledging:
        # the Leader moves its own arm on this acknowledgement.
        for action in self._logic.take_actions():
            self._execute(action)
        self._publish_status()

    def _on_leader_state(self, message: String) -> None:
        self._logic.on_leader_state(message.data, self._now())

    def _on_target(self, message: Twist) -> None:
        if any(abs(value) > 1.0e-9 for value in (
            message.linear.y, message.linear.z, message.angular.x, message.angular.y,
        )):
            self._logic.on_target_velocity(math.nan, math.nan, self._now())
            return
        self._logic.on_target_velocity(message.linear.x, message.angular.z, self._now())

    # --------------------------------------------------------- readiness
    def _update_readiness(self) -> None:
        problems: List[str] = []
        for client in (self._approach_client, self._guard_client, self._selector_client):
            if not client.service_is_ready():
                problems.append(f"{client.srv_name} unavailable")
        if self._require_gripper and self._gripper_pub.get_subscription_count() == 0:
            problems.append("no Follower Dynamixel node subscribed")
        if self.count_publishers(self._gripper_topic) > 1:
            problems.append(f"another node publishes {self._gripper_topic} (stop gripper_sequence)")
        if self.count_publishers(self._coop_topic) > 1:
            problems.append(f"another node publishes {self._coop_topic}")
        if self._needs_odom and (
            self._last_odom_time is None or self._now() - self._last_odom_time > self._odom_timeout
        ):
            problems.append("wheel odometry not received (needed for reposition)")
        self._logic.set_ready(not problems, "; ".join(problems))

    # ------------------------------------------------------------ timer
    def _on_timer(self) -> None:
        out = self._logic.update(self._now())
        for action in out.actions:
            self._execute(action)
        self._coop_pub.publish(to_twist(out.cooperation_command))
        if out.status_changed:
            self._publish_status()

    def _execute(self, action: Action) -> None:
        result = self._logic.on_action_result
        if action.kind == ActionKind.GRIPPER:
            if self._require_gripper and self._gripper_pub.get_subscription_count() == 0:
                result(action.action_id, False, "no Dynamixel subscriber", self._now())
                return
            self._gripper_pub.publish(Float64MultiArray(data=[float(x) for x in action.value]))
            self.get_logger().info(f"Follower gripper command {list(action.value)}")
            result(action.action_id, True, "published", self._now())
        elif action.kind == ActionKind.APPROACH_ENABLE:
            call_set_bool(self._approach_client, action, self._now, result, self.get_logger())
        elif action.kind == ActionKind.GUARD_ENABLE:
            call_set_bool(self._guard_client, action, self._now, result, self.get_logger())
        elif action.kind == ActionKind.SELECTOR_MODE:
            self._set_selector(action)
        else:
            result(action.action_id, False, f"unsupported action {action.kind}", self._now())

    def _set_selector(self, action: Action) -> None:
        result = self._logic.on_action_result
        client = self._selector_client
        if not client.service_is_ready():
            result(action.action_id, False, f"service {client.srv_name} unavailable", self._now())
            return
        parameter = ParameterMsg()
        parameter.name = "source_mode"
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=str(action.value)
        )
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = client.call_async(request)

        def _done(done_future) -> None:
            success, detail = False, "no response"
            try:
                response = done_future.result()
                if response is not None and response.results:
                    success = bool(response.results[0].successful)
                    detail = str(response.results[0].reason)
            except Exception as error:  # pragma: no cover - defensive
                detail = str(error)
            if not success:
                self.get_logger().error(f"selector source_mode={action.value} failed: {detail}")
            result(action.action_id, success, detail, self._now())

        future.add_done_callback(_done)

    def _publish_status(self) -> None:
        status = self._logic.status()
        self._status_pub.publish(String(data=encode_status(status)))
        if status.state != self._last_state:
            self._last_state = status.state
            log = self.get_logger().error if status.state == FollowerState.FAULT else self.get_logger().info
            log(f"Follower mission state -> {status.state.value}: {status.detail}")

    def shutdown(self) -> None:
        for _ in range(3):
            self._coop_pub.publish(to_twist(ZERO))
        for client, request in (
            (self._guard_client, SetBool.Request(data=False)),
            (self._approach_client, SetBool.Request(data=False)),
        ):
            if client.service_is_ready():
                client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[FollowerMissionNode] = None
    try:
        node = FollowerMissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.shutdown()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
