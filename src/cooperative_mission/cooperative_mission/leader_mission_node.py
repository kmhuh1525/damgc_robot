"""ROS 2 wrapper around :class:`LeaderMission` (runs on the Leader Orin).

Owns ``/leader/mission/cmd_vel_raw`` (the Leader velocity guard input in the
mission launch), ``/cooperation/target_velocity`` and the Leader Dynamixel
commands. Operator interface::

    ros2 service call /mission/start   std_srvs/srv/Trigger
    ros2 service call /mission/abort   std_srvs/srv/Trigger
    ros2 service call /mission/release std_srvs/srv/Trigger   # lower + open
    ros2 service call /mission/reset   std_srvs/srv/Trigger
"""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from std_srvs.srv import SetBool, Trigger

from .actions import Action, ActionKind
from .leader_logic import (
    COOPERATION_STATE,
    LeaderMission,
    LeaderMissionConfig,
    LeaderState,
    make_session_id,
)
from .motion import GripperParameters, SearchParameters, ZERO
from .protocol import decode_status, encode_command
from .ros_common import (
    COMMAND_QOS,
    LINK_QOS,
    SENSOR_QOS,
    STATE_QOS,
    call_set_bool,
    to_twist,
)


class LeaderMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_coordinator")
        self._declare()
        config = self._load_config()
        self._rate = float(self.get_parameter("update_rate").value)
        if not math.isfinite(self._rate) or self._rate <= 0.0:
            raise ValueError("update_rate must be positive")
        self._require_gripper = bool(self.get_parameter("require_gripper").value)
        self._logic = LeaderMission(config, make_session_id(time.time()))
        p = lambda name: str(self.get_parameter(name).value)  # noqa: E731

        self._drive_pub = self.create_publisher(Twist, p("drive_command_topic"), COMMAND_QOS)
        self._target_pub = self.create_publisher(Twist, p("target_velocity_topic"), COMMAND_QOS)
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, p("gripper_command_topic"), 10
        )
        self._follower_cmd_pub = self.create_publisher(
            String, p("follower_command_topic"), LINK_QOS
        )
        self._mission_state_pub = self.create_publisher(String, p("mission_state_topic"), STATE_QOS)
        self._coop_state_pub = self.create_publisher(String, p("cooperation_state_topic"), STATE_QOS)
        self._status_pub = self.create_publisher(String, p("status_topic"), STATE_QOS)

        self.create_subscription(Twist, p("approach_command_topic"), self._on_approach, SENSOR_QOS)
        self.create_subscription(Bool, p("detected_topic"), self._on_detected, SENSOR_QOS)
        self.create_subscription(Int32, p("tag_id_topic"), self._on_tag_id, SENSOR_QOS)
        self.create_subscription(String, p("alignment_state_topic"), self._on_alignment, SENSOR_QOS)
        self.create_subscription(String, p("gripper_status_topic"), self._on_gripper_status, 10)
        self.create_subscription(String, p("follower_status_topic"), self._on_follower_status, LINK_QOS)

        self._approach_client = self.create_client(SetBool, p("approach_enable_service"))
        self._guard_client = self.create_client(SetBool, p("guard_enable_service"))
        self._drive_topic = p("drive_command_topic")
        self._gripper_topic = p("gripper_command_topic")
        self._target_topic = p("target_velocity_topic")

        self.create_service(Trigger, "/mission/start", self._on_start)
        self.create_service(Trigger, "/mission/abort", self._on_abort)
        self.create_service(Trigger, "/mission/reset", self._on_reset)
        self.create_service(Trigger, "/mission/release", self._on_release)

        self._last_logged_state: Optional[LeaderState] = None
        self.create_timer(1.0 / self._rate, self._on_timer)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            "Leader mission ready (session %s): direction=%s speed=%.3f m/s duration=%.2f s. "
            "Call /mission/start to begin." % (
                self._logic.session, config.transport_direction,
                config.transport_speed, config.transport_duration,
            )
        )
        self._publish_state()

    # ---------------------------------------------------------- parameters
    def _declare(self) -> None:
        d = self.declare_parameter
        d("update_rate", 50.0)
        d("require_gripper", True)
        # Topics / services
        d("approach_command_topic", "/leader/approach/cmd_vel_raw")
        d("drive_command_topic", "/leader/mission/cmd_vel_raw")
        d("detected_topic", "/leader/supply/detected")
        d("tag_id_topic", "/leader/supply/tag_id")
        d("alignment_state_topic", "/leader/base_alignment/state")
        d("gripper_command_topic", "/leader/dynamixel/command")
        d("gripper_status_topic", "/leader/dynamixel/status")
        d("approach_enable_service", "/leader/approach/enable")
        d("guard_enable_service", "/leader/velocity_guard/enable")
        d("follower_command_topic", "/mission/follower_command")
        d("follower_status_topic", "/follower/mission/status")
        d("target_velocity_topic", "/cooperation/target_velocity")
        d("mission_state_topic", "/mission/state")
        d("cooperation_state_topic", "/cooperation/state")
        d("status_topic", "/leader/mission/status")
        # Perception / search
        defaults = LeaderMissionConfig()
        d("target_tag_id", defaults.target_tag_id)
        d("tag_acquire_time", defaults.tag_acquire_time)
        d("search_angular_speed", defaults.search.angular_speed)
        d("search_direction", defaults.search.direction)
        d("search_step_angle_deg", math.degrees(defaults.search.step_angle))
        d("search_dwell_time", defaults.search.dwell_time)
        d("search_max_angle_deg", math.degrees(defaults.search.max_angle))
        d("reacquire_timeout", defaults.reacquire_timeout)
        d("max_reacquire", defaults.max_reacquire)
        d("approach_timeout", defaults.approach_timeout)
        d("approach_command_timeout", defaults.approach_command_timeout)
        d("aligned_hold_time", defaults.aligned_hold_time)
        # Gripper
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
        d("lift_ack_margin", defaults.lift_ack_margin)
        d("release_lower_time", defaults.release_lower_time)
        d("release_open_settle", defaults.release_open_settle)
        # Follower coordination
        d("require_follower", defaults.require_follower)
        d("follower_status_timeout", defaults.follower_status_timeout)
        d("follower_approach_timeout", defaults.follower_approach_timeout)
        d("handshake_timeout", defaults.handshake_timeout)
        d("command_resend_period", defaults.command_resend_period)
        # Transport
        d("transport_direction", defaults.transport_direction)
        d("transport_speed", defaults.transport_speed)
        d("transport_acceleration", defaults.transport_acceleration)
        d("transport_duration", defaults.transport_duration)
        d("transport_leader_delay", defaults.transport_leader_delay)
        d("transport_settle_time", defaults.transport_settle_time)
        d("transport_status_timeout", defaults.transport_status_timeout)
        d("max_transport_speed", defaults.max_transport_speed)
        d("action_timeout", defaults.action_timeout)

    def _load_config(self) -> LeaderMissionConfig:
        v = lambda name: self.get_parameter(name).value  # noqa: E731
        return LeaderMissionConfig(
            target_tag_id=int(v("target_tag_id")),
            tag_acquire_time=float(v("tag_acquire_time")),
            search=SearchParameters(
                angular_speed=float(v("search_angular_speed")),
                direction=1.0 if float(v("search_direction")) >= 0.0 else -1.0,
                step_angle=math.radians(float(v("search_step_angle_deg"))),
                dwell_time=float(v("search_dwell_time")),
                max_angle=math.radians(float(v("search_max_angle_deg"))),
            ),
            reacquire_timeout=float(v("reacquire_timeout")),
            max_reacquire=int(v("max_reacquire")),
            approach_timeout=float(v("approach_timeout")),
            approach_command_timeout=float(v("approach_command_timeout")),
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
            lift_ack_margin=float(v("lift_ack_margin")),
            release_lower_time=float(v("release_lower_time")),
            release_open_settle=float(v("release_open_settle")),
            require_follower=bool(v("require_follower")),
            follower_status_timeout=float(v("follower_status_timeout")),
            follower_approach_timeout=float(v("follower_approach_timeout")),
            handshake_timeout=float(v("handshake_timeout")),
            command_resend_period=float(v("command_resend_period")),
            transport_direction=str(v("transport_direction")).strip().lower(),
            transport_speed=float(v("transport_speed")),
            transport_acceleration=float(v("transport_acceleration")),
            transport_duration=float(v("transport_duration")),
            transport_leader_delay=float(v("transport_leader_delay")),
            transport_settle_time=float(v("transport_settle_time")),
            transport_status_timeout=float(v("transport_status_timeout")),
            max_transport_speed=float(v("max_transport_speed")),
            action_timeout=float(v("action_timeout")),
        )

    # ------------------------------------------------------------- inputs
    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_approach(self, message: Twist) -> None:
        self._logic.on_approach_command(message.linear.x, message.angular.z, self._now())

    def _on_detected(self, message: Bool) -> None:
        self._logic.on_tag_detected(bool(message.data), self._now())

    def _on_tag_id(self, message: Int32) -> None:
        self._logic.on_tag_id(int(message.data), self._now())

    def _on_alignment(self, message: String) -> None:
        self._logic.on_alignment_state(message.data, self._now())

    def _on_gripper_status(self, message: String) -> None:
        if message.data.startswith("ERROR"):
            self.get_logger().error(f"Leader Dynamixel: {message.data}")
        self._logic.on_gripper_status(message.data, self._now())

    def _on_follower_status(self, message: String) -> None:
        status = decode_status(message.data)
        if status is None:
            self.get_logger().warning("ignored malformed Follower status", throttle_duration_sec=2.0)
            return
        self._logic.on_follower_status(status, self._now())
        if self._logic.state in (LeaderState.LIFT, LeaderState.RELEASE):
            # React to the Follower acknowledgement without waiting for the
            # next 50 Hz tick so both arms start within one DDS hop.
            self._on_timer()

    # ----------------------------------------------------------- services
    def _readiness(self) -> Tuple[bool, str]:
        problems: List[str] = []
        if not self._approach_client.service_is_ready():
            problems.append(f"{self._approach_client.srv_name} unavailable")
        if not self._guard_client.service_is_ready():
            problems.append(f"{self._guard_client.srv_name} unavailable")
        if self._require_gripper and self._gripper_pub.get_subscription_count() == 0:
            problems.append("no Leader Dynamixel node subscribed")
        if self.count_publishers(self._gripper_topic) > 1:
            problems.append(f"another node publishes {self._gripper_topic} (stop gripper_sequence)")
        if self.count_publishers(self._drive_topic) > 1:
            problems.append(f"another node publishes {self._drive_topic}")
        if self.count_publishers(self._target_topic) > 1:
            problems.append(
                f"another node publishes {self._target_topic} (stop leader_cooperation)"
            )
        return (not problems), "; ".join(problems)

    def _on_start(self, _request, response):
        ready, reason = self._readiness()
        response.success, response.message = self._logic.start(self._now(), ready, reason)
        self._log_request("start", response)
        return response

    def _on_abort(self, _request, response):
        response.success, response.message = self._logic.abort(self._now())
        self._log_request("abort", response)
        return response

    def _on_reset(self, _request, response):
        response.success, response.message = self._logic.reset(self._now())
        self._log_request("reset", response)
        return response

    def _on_release(self, _request, response):
        response.success, response.message = self._logic.release(self._now())
        self._log_request("release", response)
        return response

    def _log_request(self, name: str, response) -> None:
        log = self.get_logger().info if response.success else self.get_logger().warning
        log(f"/mission/{name}: {response.message}")

    # ------------------------------------------------------------ timer
    def _on_timer(self) -> None:
        now = self._now()
        out = self._logic.update(now)
        for action in out.actions:
            self._execute(action)
        self._drive_pub.publish(to_twist(out.drive))
        self._target_pub.publish(to_twist(out.target_velocity))
        if out.follower_message is not None:
            self._follower_cmd_pub.publish(String(data=encode_command(out.follower_message)))
        if out.state_changed:
            self._publish_state()

    def _execute(self, action: Action) -> None:
        result = self._logic.on_action_result
        if action.kind == ActionKind.GRIPPER:
            if self._require_gripper and self._gripper_pub.get_subscription_count() == 0:
                result(action.action_id, False, "no Dynamixel subscriber", self._now())
                return
            self._gripper_pub.publish(Float64MultiArray(data=[float(x) for x in action.value]))
            self.get_logger().info(f"Leader gripper command {list(action.value)}")
            result(action.action_id, True, "published", self._now())
        elif action.kind == ActionKind.APPROACH_ENABLE:
            call_set_bool(self._approach_client, action, self._now, result, self.get_logger())
        elif action.kind == ActionKind.GUARD_ENABLE:
            call_set_bool(self._guard_client, action, self._now, result, self.get_logger())
        else:
            result(action.action_id, False, f"unsupported action {action.kind}", self._now())

    def _publish_state(self) -> None:
        state = self._logic.state
        self._mission_state_pub.publish(String(data=state.value))
        self._coop_state_pub.publish(String(data=COOPERATION_STATE[state]))
        follower = self._logic.follower_status
        self._status_pub.publish(String(data=json.dumps({
            "session": self._logic.session,
            "state": state.value,
            "detail": self._logic.detail,
            "follower_state": follower.state.value if follower else None,
            "follower_detail": follower.detail if follower else None,
        }, ensure_ascii=False)))
        if state != self._last_logged_state:
            self._last_logged_state = state
            log = self.get_logger().error if state == LeaderState.FAULT else self.get_logger().info
            log(f"mission state -> {state.value}: {self._logic.detail}")

    def shutdown(self) -> None:
        now = self._now()
        if self._logic.state not in (LeaderState.IDLE, LeaderState.DONE, LeaderState.FAULT):
            self._logic.abort(now, "Leader mission node shutting down")
        out = self._logic.update(now)
        if out.follower_message is not None:
            self._follower_cmd_pub.publish(String(data=encode_command(out.follower_message)))
        for _ in range(3):
            self._drive_pub.publish(to_twist(ZERO))
            self._target_pub.publish(to_twist(ZERO))
        if self._guard_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._guard_client.call_async(request)
        self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[LeaderMissionNode] = None
    try:
        node = LeaderMissionNode()
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
