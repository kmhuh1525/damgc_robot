"""Publish conservative raw Follower commands from atomic alignment samples."""

import time
from math import isfinite, sqrt
from typing import List, Optional

import rclpy
from follower_alignment_msgs.msg import FollowerAlignmentCommand
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

from follower_approach_control.approach_controller_logic import (
    BLIND_FINAL_APPROACH,
    FINAL_APPROACH,
    KNOWN_MODES,
    KNOWN_STATES,
    TAG_LOST,
    BaseControlMeasurement,
    ControllerParameters,
    PlanarCommand,
    compute_approach_command,
    quaternion_yaw,
    sample_is_fresh,
)


INPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ApproachControllerNode(Node):
    """Consume one coherent target/mode/state command and publish raw Twist."""

    def __init__(self) -> None:
        super().__init__("approach_controller")
        self._declare_parameters()
        self._load_and_validate_parameters()
        self._raw_pub = self.create_publisher(Twist, "approach/cmd_vel_raw", 10)
        self._enabled_pub = self.create_publisher(Bool, "approach/enabled", 10)
        self._command_sub = self.create_subscription(
            FollowerAlignmentCommand,
            "alignment/command",
            self._on_command,
            INPUT_QOS,
        )
        self._detected_sub = self.create_subscription(
            Bool, "supply/detected", self._on_detected, INPUT_QOS
        )
        self._tag_id_sub = self.create_subscription(
            Int32, "supply/tag_id", self._on_tag_id, INPUT_QOS
        )
        self._enable_service = self.create_service(
            SetBool, "approach/enable", self._on_enable
        )
        self._detected = False
        self._selected_tag_id = -1
        self._measurement: Optional[BaseControlMeasurement] = None
        self._command_stamp_seconds: Optional[float] = None
        self._command_received_seconds: Optional[float] = None
        self._command_state: Optional[str] = None
        self._command_mode: Optional[str] = None
        self._enabled_pub.publish(Bool(data=self._enabled))
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _declare_parameters(self) -> None:
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("enabled_on_startup", False)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("pose_timeout", 1.20)
        # Deprecated no-op retained for compatibility with existing overrides.
        self.declare_parameter("sample_sync_tolerance", 0.10)
        # Retained for launch/config compatibility; perception owns final distance.
        self.declare_parameter("target_forward", 0.242)
        self.declare_parameter("linear_gain", 0.20)
        self.declare_parameter("angular_gain", 0.80)
        self.declare_parameter("lateral_gain", 0.50)
        self.declare_parameter("max_raw_linear_speed", 0.04)
        self.declare_parameter("max_raw_angular_speed", 0.15)
        self.declare_parameter("near_max_angular_speed", 0.10)
        self.declare_parameter("max_final_linear_speed", 0.02)
        self.declare_parameter("max_final_angular_speed", 0.06)
        self.declare_parameter("blind_final_speed", 0.015)
        # Deprecated compatibility parameter; hybrid control remains forward-only.
        self.declare_parameter("allow_reverse", False)

    def _load_and_validate_parameters(self) -> None:
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._target_tag_id = int(self.get_parameter("target_tag_id").value)
        self._enabled = bool(self.get_parameter("enabled_on_startup").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout").value)
        self._sample_sync_tolerance = float(
            self.get_parameter("sample_sync_tolerance").value
        )
        self._target_forward = float(self.get_parameter("target_forward").value)
        self._allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self._control_parameters = ControllerParameters(
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            lateral_gain=float(self.get_parameter("lateral_gain").value),
            max_raw_linear_speed=float(
                self.get_parameter("max_raw_linear_speed").value
            ),
            max_raw_angular_speed=float(
                self.get_parameter("max_raw_angular_speed").value
            ),
            near_max_angular_speed=float(
                self.get_parameter("near_max_angular_speed").value
            ),
            max_final_linear_speed=float(
                self.get_parameter("max_final_linear_speed").value
            ),
            max_final_angular_speed=float(
                self.get_parameter("max_final_angular_speed").value
            ),
            blind_final_speed=float(self.get_parameter("blind_final_speed").value),
        )
        if not self._base_frame:
            raise ValueError("base_frame must not be empty")
        if self._target_tag_id < 0:
            raise ValueError("target_tag_id must be non-negative")
        if not all(
            isfinite(value)
            for value in (
                self._publish_rate,
                self._pose_timeout,
                self._sample_sync_tolerance,
                self._target_forward,
            )
        ):
            raise ValueError("Controller timing and target parameters must be finite")
        if self._publish_rate <= 0.0 or self._pose_timeout <= 0.0:
            raise ValueError("publish_rate and pose_timeout must be positive")
        if self._sample_sync_tolerance < 0.0:
            raise ValueError("sample_sync_tolerance must not be negative")
        if self._target_forward <= 0.0:
            raise ValueError("target_forward must be positive")
        self._control_parameters.validate()

    def _on_detected(self, message: Bool) -> None:
        detected = bool(message.data)
        if detected != self._detected:
            self._invalidate_sample()
        self._detected = detected

    def _on_tag_id(self, message: Int32) -> None:
        tag_id = int(message.data)
        if tag_id != self._selected_tag_id:
            self._invalidate_sample()
        self._selected_tag_id = tag_id

    def _on_command(self, message: FollowerAlignmentCommand) -> None:
        received_seconds = time.monotonic()
        now_seconds = self.get_clock().now().nanoseconds / 1.0e9
        stamp_seconds = message.header.stamp.sec + message.header.stamp.nanosec / 1.0e9
        blind = (
            message.control_mode == BLIND_FINAL_APPROACH
            and message.alignment_state == FINAL_APPROACH
        )
        if (
            (not blind and not self._detected)
            or (not blind and self._selected_tag_id != self._target_tag_id)
            or message.header.frame_id != self._base_frame
            or not sample_is_fresh(
                now_seconds,
                stamp_seconds,
                received_seconds,
                received_seconds,
                self._pose_timeout,
            )
            or message.control_mode not in KNOWN_MODES
            or message.alignment_state not in KNOWN_STATES
            or not self._pose_is_valid(message.target_pose)
        ):
            self._invalidate_sample()
            return
        pose = message.target_pose
        yaw = quaternion_yaw(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        if yaw is None:
            self._invalidate_sample()
            return
        self._measurement = BaseControlMeasurement(
            float(pose.position.x), float(pose.position.y), yaw
        )
        self._command_stamp_seconds = stamp_seconds
        self._command_received_seconds = received_seconds
        self._command_mode = str(message.control_mode)
        self._command_state = str(message.alignment_state)

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._enabled = bool(request.data)
        self._invalidate_sample()
        self._enabled_pub.publish(Bool(data=self._enabled))
        response.success = True
        response.message = (
            "approach controller enabled; waiting for a fresh coherent sample"
            if self._enabled
            else "approach controller disabled"
        )
        self._raw_pub.publish(Twist())
        return response

    def _on_timer(self) -> None:
        now_seconds = self.get_clock().now().nanoseconds / 1.0e9
        receipt_now = time.monotonic()
        fresh = (
            self._command_stamp_seconds is not None
            and self._command_received_seconds is not None
            and sample_is_fresh(
                now_seconds,
                self._command_stamp_seconds,
                receipt_now,
                self._command_received_seconds,
                self._pose_timeout,
            )
        )
        command = compute_approach_command(
            self._command_state or TAG_LOST,
            self._command_mode or TAG_LOST,
            self._measurement,
            self._control_parameters,
            enabled=self._enabled,
            detected=self._detected,
            tag_valid=self._selected_tag_id == self._target_tag_id,
            fresh=fresh,
            coherent=self._measurement is not None,
        )
        self._raw_pub.publish(self._to_twist(command))

    def _invalidate_sample(self) -> None:
        self._measurement = None
        self._command_stamp_seconds = None
        self._command_received_seconds = None
        self._command_state = None
        self._command_mode = None

    @staticmethod
    def _pose_is_valid(message) -> bool:
        values = (
            message.position.x,
            message.position.y,
            message.position.z,
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        if not all(isfinite(value) for value in values):
            return False
        norm = sqrt(
            message.orientation.x**2
            + message.orientation.y**2
            + message.orientation.z**2
            + message.orientation.w**2
        )
        return isfinite(norm) and norm > 1.0e-12

    @staticmethod
    def _to_twist(command: PlanarCommand) -> Twist:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def stop(self) -> None:
        self._raw_pub.publish(Twist())


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ApproachControllerNode] = None
    try:
        node = ApproachControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.stop()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
