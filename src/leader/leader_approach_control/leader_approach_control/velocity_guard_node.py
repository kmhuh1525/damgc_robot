"""Leader AprilTag velocity guard with watchdog and slew limits."""

import time
from math import isfinite
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from leader_approach_control.velocity_guard_logic import (
    GuardParameters,
    PlanarVelocity,
    apply_slew_limit,
    command_is_fresh,
    sanitize_velocity,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class VelocityGuardNode(Node):
    """Publish validated and rate-limited commands for APPROACH selection."""

    def __init__(self) -> None:
        super().__init__("velocity_guard")
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._safe_pub = self.create_publisher(
            Twist, self._safe_command_topic, COMMAND_QOS
        )
        self._raw_sub = self.create_subscription(
            Twist, self._command_topic, self._on_command, COMMAND_QOS
        )
        self._enable_service = self.create_service(
            SetBool, "velocity_guard/enable", self._on_enable
        )
        self._last_command: Optional[PlanarVelocity] = None
        self._last_received_seconds: Optional[float] = None
        self._last_output = PlanarVelocity()
        self._last_output_seconds: Optional[float] = None
        self._timer = self.create_timer(
            1.0 / self._publish_rate, self._on_timer
        )
        self.get_logger().info(
            "Velocity guard ready: enabled=%s, timeout=%.3fs, limits=(%.3fm/s, "
            "%.3frad/s)"
            % (
                self._enabled,
                self._guard_parameters.command_timeout,
                self._guard_parameters.max_linear_speed,
                self._guard_parameters.max_angular_speed,
            )
        )

    def _declare_parameters(self) -> None:
        """Declare startup-only guard parameters."""
        self.declare_parameter("guard_enabled_on_startup", False)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("command_timeout", 0.30)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.20)
        self.declare_parameter("max_linear_acceleration", 0.10)
        self.declare_parameter("max_angular_acceleration", 0.40)
        self.declare_parameter("max_slew_dt", 0.10)
        self.declare_parameter("axis_epsilon", 1.0e-9)
        self.declare_parameter("shutdown_stop_count", 3)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter(
            "command_topic", "/leader/approach/cmd_vel_raw"
        )
        self.declare_parameter(
            "safe_command_topic", "/leader/approach/cmd_vel_safe"
        )

    def _load_and_validate_parameters(self) -> None:
        """Load and validate finite timing, topic, and velocity parameters."""
        self._enabled = bool(
            self.get_parameter("guard_enabled_on_startup").value
        )
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._safe_command_topic = str(
            self.get_parameter("safe_command_topic").value
        )
        self._shutdown_stop_count = int(
            self.get_parameter("shutdown_stop_count").value
        )
        self._guard_parameters = GuardParameters(
            max_linear_speed=float(
                self.get_parameter("max_linear_speed").value
            ),
            max_angular_speed=float(
                self.get_parameter("max_angular_speed").value
            ),
            max_linear_acceleration=float(
                self.get_parameter("max_linear_acceleration").value
            ),
            max_angular_acceleration=float(
                self.get_parameter("max_angular_acceleration").value
            ),
            command_timeout=float(
                self.get_parameter("command_timeout").value
            ),
            max_slew_dt=float(self.get_parameter("max_slew_dt").value),
            axis_epsilon=float(self.get_parameter("axis_epsilon").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
        )
        if not self._command_topic or not self._safe_command_topic:
            raise ValueError("Command topics must not be empty")
        if not isfinite_positive(self._publish_rate):
            raise ValueError("publish_rate must be finite and greater than zero")
        if self._shutdown_stop_count < 1:
            raise ValueError("shutdown_stop_count must be at least one")
        self._guard_parameters.validate()

    def _on_command(self, message: Twist) -> None:
        """Validate each raw command and discard invalid data immediately."""
        command = sanitize_velocity(
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
            self._guard_parameters,
        )
        now_seconds = time.monotonic()
        if command is None:
            self._force_zero(now_seconds)
            self.get_logger().warning(
                "Rejected invalid raw velocity command",
                throttle_duration_sec=1.0,
            )
            return
        self._last_command = command
        self._last_received_seconds = now_seconds

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Enable or disable the final output and clear stale command history."""
        self._enabled = bool(request.data)
        self._last_command = None
        self._last_received_seconds = None
        self._force_zero(time.monotonic())
        response.success = True
        response.message = (
            "velocity guard enabled; waiting for a fresh raw command"
            if self._enabled
            else "velocity guard disabled"
        )
        self.get_logger().info(response.message)
        return response

    def _on_timer(self) -> None:
        """Publish a clamped/slew-limited command or an explicit zero."""
        now_seconds = time.monotonic()
        if (
            not self._enabled
            or self._last_command is None
            or self._last_received_seconds is None
            or not command_is_fresh(
                now_seconds,
                self._last_received_seconds,
                self._guard_parameters.command_timeout,
            )
        ):
            self._force_zero(now_seconds)
            return

        if self._last_output_seconds is None:
            self._last_output_seconds = now_seconds
            self._last_output = PlanarVelocity()
        else:
            elapsed = now_seconds - self._last_output_seconds
            self._last_output = apply_slew_limit(
                self._last_output,
                self._last_command,
                elapsed,
                self._guard_parameters,
            )
            self._last_output_seconds = now_seconds
        self._safe_pub.publish(self._to_twist(self._last_output))

    def _force_zero(self, now_seconds: float) -> None:
        """Reset output state and publish immediate zero for safety transitions."""
        self._last_output = PlanarVelocity()
        self._last_output_seconds = now_seconds
        self._safe_pub.publish(Twist())

    @staticmethod
    def _to_twist(command: PlanarVelocity) -> Twist:
        """Populate only linear.x and angular.z."""
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def stop(self) -> None:
        """Publish a short zero burst before shutdown."""
        stop = Twist()
        for _ in range(self._shutdown_stop_count):
            self._safe_pub.publish(stop)


def isfinite_positive(value: float) -> bool:
    """Return whether a scalar is finite and strictly positive."""
    return isfinite(value) and value > 0.0


def main(args: Optional[List[str]] = None) -> None:
    """Initialize ROS 2 and spin the velocity guard."""
    rclpy.init(args=args)
    node: Optional[VelocityGuardNode] = None
    try:
        node = VelocityGuardNode()
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
