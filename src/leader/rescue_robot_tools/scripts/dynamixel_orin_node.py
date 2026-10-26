#!/usr/bin/env python3
"""ROS2 open-loop gripper controller for RX-64/RX-28 through U2D2.

Raw command topic: /<namespace>/dynamixel/command
Message: std_msgs/msg/Float64MultiArray, data=[rx64_raw, rx28_raw, torque]
Optional targeted form: [rx64_raw, rx28_raw, rx64_torque, rx28_torque]

Semantic command topic: /<namespace>/gripper/command
Message: std_msgs/msg/String, data=open|close|middle|rx64_high|rx64_low|rx64_middle|stop

Use -1 for a position or torque field that must remain unchanged.
Example: [450.0, 1.0, 1.0] enables torque and sends both positions.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from dynamixel_orin import DynamixelCommunicationError, DynamixelOrin, get_profile


class DynamixelOrinNode(Node):
    def __init__(self):
        super().__init__("dynamixel_orin_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("robot", "leader")
        self.declare_parameter("rx64_speed", 10)
        self.declare_parameter("rx28_speed", 40)
        self.declare_parameter("lift_enabled", False)
        # Safe idle pose used immediately after the Dynamixel bus is opened.
        self.declare_parameter("startup_rx64_raw", 600)
        self.declare_parameter("startup_rx28_raw", 500)
        self.declare_parameter("startup_torque", True)
        self.declare_parameter("startup_pose_enabled", True)
        self.declare_parameter("shutdown_torque_off", True)

        robot = str(self.get_parameter("robot").value)
        self.profile = get_profile(robot)
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        rx64_speed = int(self.get_parameter("rx64_speed").value)
        rx28_speed = int(self.get_parameter("rx28_speed").value)
        startup_rx64 = int(self.get_parameter("startup_rx64_raw").value)
        startup_rx28 = int(self.get_parameter("startup_rx28_raw").value)
        startup_torque = bool(self.get_parameter("startup_torque").value)
        startup_pose_enabled = bool(
            self.get_parameter("startup_pose_enabled").value
        )
        self.controller = None

        try:
            self.controller = DynamixelOrin(port, self.profile, baudrate)
            if bool(self.get_parameter("lift_enabled").value):
                self._apply_rx64_speed(rx64_speed)
            self._apply_rx28_speed(rx28_speed)
            self.publish_status(
                f"READY robot={robot} port={port} baudrate={baudrate} "
                f"rx64_speed={rx64_speed} rx28_speed={rx28_speed}"
            )
            self._apply_startup_configuration(
                startup_pose_enabled,
                startup_torque,
                startup_rx64,
                startup_rx28,
            )
        except Exception as exc:
            if self.controller is not None:
                self.controller.close()
                self.controller = None
            self.publish_status(f"ERROR opening/configuring U2D2: {exc}")
            self.get_logger().error(str(exc))

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "dynamixel/command",
            self.command_callback,
            10,
        )
        self.gripper_subscription = self.create_subscription(
            String,
            "gripper/command",
            self.gripper_command_callback,
            10,
        )
        self.status_publisher = self.create_publisher(String, "dynamixel/status", 10)

    def _apply_rx64_speed(self, rx64_speed: int) -> None:
        """Configure RX-64 speed once without commanding torque or position."""
        self.controller.set_rx64_speed(rx64_speed)
        self.publish_status(f"RX64_SPEED raw={rx64_speed}")

    def _apply_rx28_speed(self, rx28_speed: int) -> None:
        """Configure RX-28 speed once without commanding torque or position."""
        self.controller.set_rx28_speed(rx28_speed)
        self.publish_status(f"RX28_SPEED raw={rx28_speed}")

    def _apply_startup_configuration(
        self,
        startup_pose_enabled: bool,
        startup_torque: bool,
        startup_rx64: int,
        startup_rx28: int,
    ) -> None:
        """Apply the optional startup torque and pose without implicit writes."""
        if startup_pose_enabled and bool(self.get_parameter("lift_enabled").value) and not (
            self.profile["rx64_min"] <= startup_rx64 <= self.profile["rx64_max"]
            and self.profile["rx28_min"] <= startup_rx28 <= self.profile["rx28_max"]
        ):
            raise ValueError(
                "startup positions are outside the configured Dynamixel limits"
            )

        if startup_torque:
            if bool(self.get_parameter("lift_enabled").value):
                self.controller.set_torque(self.profile["rx64_id"], True)
            self.controller.set_torque(self.profile["rx28_id"], True)

        if startup_pose_enabled:
            if bool(self.get_parameter("lift_enabled").value):
                self.controller.set_position(
                    self.profile["rx64_id"], startup_rx64,
                    self.profile["rx64_min"], self.profile["rx64_max"]
                )
            self.controller.set_position(
                self.profile["rx28_id"], startup_rx28,
                self.profile["rx28_min"], self.profile["rx28_max"]
            )
            self.publish_status(
                f"STARTUP idle pose rx64={startup_rx64} rx28={startup_rx28} "
                f"torque={int(startup_torque)}"
            )
        else:
            self.publish_status(
                "STARTUP pose disabled torque=%d" % int(startup_torque)
            )

    def gripper_command_callback(self, message: String):
        """Handle simple semantic commands without exposing raw positions."""
        if self.controller is None:
            self.publish_status("ERROR controller is not connected")
            return

        command = message.data.strip().lower()
        rx28_positions = {
            "open": self.profile["rx28_max"],
            "close": self.profile["rx28_min"],
            "middle": round((self.profile["rx28_min"] + self.profile["rx28_max"]) / 2),
        }
        rx64_positions = {
            "rx64_high": self.profile["rx64_min"],
            "rx64_low": self.profile["rx64_max"],
            "rx64_middle": round((self.profile["rx64_min"] + self.profile["rx64_max"]) / 2),
        }

        try:
            if command in rx28_positions:
                self.controller.set_torque(self.profile["rx28_id"], True)
                self.controller.set_position(
                    self.profile["rx28_id"],
                    rx28_positions[command],
                    self.profile["rx28_min"],
                    self.profile["rx28_max"],
                )
                self.publish_status(
                    f"OK RX28 position={rx28_positions[command]} command={command}"
                )
            elif command in rx64_positions:
                if not bool(self.get_parameter("lift_enabled").value):
                    self.publish_status("IGNORED lift disabled")
                    return
                self.controller.set_torque(self.profile["rx64_id"], True)
                self.controller.set_position(
                    self.profile["rx64_id"],
                    rx64_positions[command],
                    self.profile["rx64_min"],
                    self.profile["rx64_max"],
                )
                self.publish_status(
                    f"OK RX64 position={rx64_positions[command]} command={command}"
                )
            elif command == "stop":
                self.controller.set_torque(self.profile["rx64_id"], False)
                self.controller.set_torque(self.profile["rx28_id"], False)
                self.publish_status("OK command=stop torque=0")
            else:
                self.publish_status(
                    "ERROR command must be open, close, middle, "
                    "rx64_high, rx64_low, rx64_middle, or stop"
                )
        except DynamixelCommunicationError as exc:
            self.publish_status(f"ERROR {exc}")
        except Exception as exc:
            self.publish_status(f"ERROR command failed: {exc}")

    def publish_status(self, message: str):
        self.get_logger().info(message)
        if hasattr(self, "status_publisher"):
            self.status_publisher.publish(String(data=message))

    def command_callback(self, message: Float64MultiArray):
        if self.controller is None:
            self.publish_status("ERROR controller is not connected")
            return
        if len(message.data) not in (3, 4):
            self.publish_status(
                "ERROR command must be [rx64_raw, rx28_raw, torque] or "
                "[rx64_raw, rx28_raw, rx64_torque, rx28_torque]"
            )
            return

        values = message.data[:4]
        if not all(math.isfinite(value) for value in values):
            self.publish_status("ERROR command contains NaN or infinity")
            return

        rx64_raw, rx28_raw = values[:2]
        targeted_torque = len(values) == 4
        if targeted_torque:
            rx64_torque, rx28_torque = values[2:4]
        else:
            torque = values[2]

        if not bool(self.get_parameter("lift_enabled").value):
            rx64_raw = -1.0
            if targeted_torque:
                rx64_torque = -1.0
            else:
                rx64_torque, rx28_torque = -1.0, torque
                targeted_torque = True

        try:
            if targeted_torque:
                if rx64_torque >= 0:
                    self.controller.set_torque(
                        self.profile["rx64_id"], bool(round(rx64_torque))
                    )
                if rx28_torque >= 0:
                    self.controller.set_torque(
                        self.profile["rx28_id"], bool(round(rx28_torque))
                    )
            elif torque >= 0:
                enabled = bool(round(torque))
                self.controller.set_torque(self.profile["rx64_id"], enabled)
                self.controller.set_torque(self.profile["rx28_id"], enabled)

            if rx64_raw >= 0:
                self.controller.set_position(
                    self.profile["rx64_id"],
                    round(rx64_raw),
                    self.profile["rx64_min"],
                    self.profile["rx64_max"],
                )

            if rx28_raw >= 0:
                self.controller.set_position(
                    self.profile["rx28_id"],
                    round(rx28_raw),
                    self.profile["rx28_min"],
                    self.profile["rx28_max"],
                )

            self.publish_status(
                "OK rx64=%d rx28=%d %s"
                % (
                    round(rx64_raw),
                    round(rx28_raw),
                    (
                        "rx64_torque=%d rx28_torque=%d"
                        % (round(rx64_torque), round(rx28_torque))
                        if targeted_torque
                        else "torque=%d" % round(torque)
                    ),
                )
            )
        except DynamixelCommunicationError as exc:
            self.publish_status(f"ERROR {exc}")
        except Exception as exc:
            self.publish_status(f"ERROR command failed: {exc}")

    def destroy_node(self):
        if self.controller is not None:
            if bool(self.get_parameter("shutdown_torque_off").value):
                try:
                    self.controller.set_torque(self.profile["rx64_id"], False)
                    self.controller.set_torque(self.profile["rx28_id"], False)
                except Exception as exc:
                    self.get_logger().error(f"failed to disable torque: {exc}")
            self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamixelOrinNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
