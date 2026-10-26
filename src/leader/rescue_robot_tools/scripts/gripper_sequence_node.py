#!/usr/bin/env python3
"""Open on AprilTag detection and close when alignment becomes ALIGNED."""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from dynamixel_orin import get_profile


class SequenceState(str, Enum):
    WAITING_FOR_TAG = "WAITING_FOR_TAG"
    OPENING = "OPENING"
    CLOSING = "CLOSING"
    LIFTING = "LIFTING"
    DONE = "DONE"


class GripperSequenceNode(Node):
    """Connect the existing AprilTag state outputs to Dynamixel commands."""

    def __init__(self) -> None:
        super().__init__("gripper_sequence")
        self.declare_parameter("detection_topic", "/leader/supply/detected")
        self.declare_parameter("alignment_topic", "/leader/base_alignment/state")
        self.declare_parameter("raw_command_topic", "/leader/dynamixel/command")
        self.declare_parameter("gripper_topic", "/leader/gripper/command")
        self.declare_parameter("open_raw", 950)
        self.declare_parameter("close_raw", 350)
        self.declare_parameter("close_wait", 3.0)
        self.declare_parameter("enabled", False)
        self.declare_parameter("task_mode", False)
        self.declare_parameter("open_wait", 17.0)
        self.declare_parameter("task_timeout", 120.0)
        self.declare_parameter("grasp_distance", 0.20)
        self.declare_parameter("grasp_distance_tolerance", 0.02)
        self.declare_parameter("robot", "leader")
        self.declare_parameter("lift_enabled", False)
        self.declare_parameter("lift_raw", -1.0)
        self.declare_parameter("lost_rx64_raw", 600)
        self.declare_parameter("lost_rx28_raw", 500)
        self.declare_parameter("tag_lost_idle_enabled", True)

        detection_topic = str(self.get_parameter("detection_topic").value)
        alignment_topic = str(self.get_parameter("alignment_topic").value)
        raw_topic = str(self.get_parameter("raw_command_topic").value)
        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self._open_raw = int(self.get_parameter("open_raw").value)
        self._close_raw = int(self.get_parameter("close_raw").value)
        self._close_wait = float(self.get_parameter("close_wait").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._lift_enabled = bool(self.get_parameter("lift_enabled").value)
        self._lost_rx64_raw = int(self.get_parameter("lost_rx64_raw").value)
        self._lost_rx28_raw = int(self.get_parameter("lost_rx28_raw").value)
        self._tag_lost_idle_enabled = bool(
            self.get_parameter("tag_lost_idle_enabled").value
        )
        # RX-64 remains disabled by default until its mechanical direction is validated.
        lift_raw_value = self.get_parameter("lift_raw").value
        try:
            self._lift_raw_value = float(lift_raw_value)
        except (TypeError, ValueError):
            self._lift_raw_value = math.nan
        self._lift_raw = (
            int(self._lift_raw_value)
            if math.isfinite(self._lift_raw_value)
            and self._lift_raw_value.is_integer()
            else -1
        )
        robot = str(self.get_parameter("robot").value)
        profile = get_profile(robot)
        self._lift_min_raw = int(profile["rx64_min"])
        self._lift_max_raw = int(profile["rx64_max"])
        self._lift_raw_valid = (
            math.isfinite(self._lift_raw_value)
            and self._lift_raw_value.is_integer()
            and self._lift_min_raw <= self._lift_raw <= self._lift_max_raw
        )

        if not 1 <= self._open_raw <= 1021:
            raise ValueError("open_raw must be between 1 and 1021")
        if not 1 <= self._close_raw <= 1021:
            raise ValueError("close_raw must be between 1 and 1021")
        if (
            self._tag_lost_idle_enabled
            and not self._lift_min_raw <= self._lost_rx64_raw <= self._lift_max_raw
        ):
            raise ValueError("lost_rx64_raw is outside the configured RX-64 limits")
        if self._tag_lost_idle_enabled and not 1 <= self._lost_rx28_raw <= 1021:
            raise ValueError("lost_rx28_raw must be between 1 and 1021")

        self._raw_pub = self.create_publisher(Float64MultiArray, raw_topic, 10)
        self._gripper_pub = self.create_publisher(String, gripper_topic, 10)
        self._status_pub = self.create_publisher(String, "sequence/status", 10)
        self.create_subscription(Bool, detection_topic, self._detection_callback, 10)
        self.create_subscription(String, alignment_topic, self._alignment_callback, 10)
        self._task_mode = bool(self.get_parameter("task_mode").value)
        self._task_phase = "IDLE"
        self._task_queue = []
        self._task_future = None
        self._task_next = ""
        self._alignment_received = 0.0
        self._aligned_samples = 0
        if self._task_mode:
            self._approach_client = self.create_client(SetBool, "approach/enable")
            self._guard_client = self.create_client(SetBool, "velocity_guard/enable")
            selector = self.get_namespace().rstrip("/") + "/command_selector"
            self._selector_client = self.create_client(SetParameters, selector + "/set_parameters")
            self._motor_ack = False
            self.create_subscription(String, raw_topic.rsplit("/", 1)[0] + "/status", self._task_motor_status, 10)
            self._distance_pose = None
            self._distance_received = 0.0
            self.create_subscription(PoseStamped, "supply/relative_pose", self._task_pose_callback, 10)
            self.create_service(Trigger, "grasp/start", self._start_task)
            self.create_service(Trigger, "grasp/cancel", self._cancel_task)
        self._timer = self.create_timer(0.05, self._on_timer)

        self._state = SequenceState.WAITING_FOR_TAG
        self._tag_detected = False
        self._alignment_state = "TAG_LOST"
        self._deadline: Optional[float] = None
        self._publish_status(
            "WAITING_FOR_TAG enabled=%s detection=%s alignment=%s open_raw=%d "
            "close_raw=%d lift_enabled=%s lift_raw=%s"
            % (
                self._enabled,
                detection_topic,
                alignment_topic,
                self._open_raw,
                self._close_raw,
                self._lift_enabled,
                lift_raw_value,
            )
        )
        if self._lift_enabled and not self._lift_raw_valid:
            self._publish_status(
                "ERROR invalid lift_raw=%s; expected integer %d..%d; RX-64 lift disabled"
                % (lift_raw_value, self._lift_min_raw, self._lift_max_raw)
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_raw(
        self,
        rx64: float,
        rx28: float,
        torque: float,
        rx64_torque: Optional[float] = None,
        rx28_torque: Optional[float] = None,
    ) -> None:
        if rx64_torque is not None and rx28_torque is not None:
            data = [rx64, rx28, rx64_torque, rx28_torque]
        else:
            data = [rx64, rx28, torque]
        self._raw_pub.publish(Float64MultiArray(data=data))
        self.get_logger().info(
            "raw command rx64=%s rx28=%s torque=%s" % (rx64, rx28, torque)
        )

    def _publish_gripper(self, command: str) -> None:
        self._gripper_pub.publish(String(data=command))
        self.get_logger().info("gripper command: %s" % command)

    def _detection_callback(self, message: Bool) -> None:
        if not self._enabled:
            return
        if getattr(self, "_task_mode", False):
            self._tag_detected = bool(message.data)
            return
        detected = bool(message.data)
        was_detected = self._tag_detected
        self._tag_detected = detected
        if not detected and was_detected and self._tag_lost_idle_enabled:
            self._publish_raw(
                float(self._lost_rx64_raw), float(self._lost_rx28_raw),
                -1.0, 1.0, 1.0
            )
            self._publish_status(
                "TAG_LOST idle pose rx64=%d rx28=%d"
                % (self._lost_rx64_raw, self._lost_rx28_raw)
            )
        if detected and self._state == SequenceState.WAITING_FOR_TAG:
            self._state = SequenceState.OPENING
            self._publish_raw(-1.0, float(self._open_raw), -1.0, -1.0, 1.0)
            self._publish_status("OPENING tag_detected")

    def _alignment_callback(self, message: String) -> None:
        self._alignment_state = message.data.strip().upper()
        if getattr(self, "_task_mode", False):
            self._alignment_received = self._now()
            self._aligned_samples = self._aligned_samples + 1 if self._alignment_state == "ALIGNED" else 0

    def _on_timer(self) -> None:
        if getattr(self, "_task_mode", False):
            self._task_timer()
            return
        if not self._enabled:
            return

        if self._state == SequenceState.OPENING:
            if self._alignment_state == "ALIGNED":
                self._state = SequenceState.CLOSING
                self._publish_raw(-1.0, float(self._close_raw), -1.0, -1.0, 1.0)
                self._publish_status("CLOSING alignment=ALIGNED close_raw=%d" % self._close_raw)
                if self._lift_enabled and self._lift_raw_valid:
                    self._deadline = self._now() + self._close_wait
                else:
                    self._state = SequenceState.DONE
                    self._deadline = None
                    if self._lift_enabled:
                        self._publish_status("DONE lift_skipped_invalid_raw")
                    else:
                        self._publish_status("DONE lift_disabled")
                    return
            return

        if self._state == SequenceState.CLOSING:
            if self._deadline is not None and self._now() >= self._deadline:
                self._state = SequenceState.LIFTING
                self._publish_raw(float(self._lift_raw), -1.0, -1.0, 1.0, -1.0)
                self._deadline = None
                self._publish_status("LIFTING lift_raw=%d" % self._lift_raw)
            return

        if self._state == SequenceState.LIFTING:
            self._state = SequenceState.DONE
            self._publish_status("DONE lift_raw=%d" % self._lift_raw)
            return

        if (
            self._state == SequenceState.DONE
            and not self._tag_detected
            and self._alignment_state != "ALIGNED"
        ):
            self._state = SequenceState.WAITING_FOR_TAG
            self._publish_status("WAITING_FOR_TAG next cycle armed")


    def _queue_task_controls(self, controls, next_phase):
        self._task_queue = list(controls)
        self._task_future = None
        self._task_next = next_phase
        self._task_phase = "CONTROLS"

    def _stop_task_controls(self, next_phase):
        controls = [("guard", False), ("selector", "STOP")]
        if next_phase != "CLOSE":
            controls.append(("approach", False))
        self._queue_task_controls(controls, next_phase)

    def _task_motor_status(self, message):
        if self._task_phase in ("IDLE", "DONE", "CANCELLED", "FAULT"):
            return
        if message.data.startswith("ERROR"):
            self._task_fault("motor: " + message.data)
        elif self._task_phase in ("WAIT_OPEN", "WAIT_CLOSE"):
            target = self._open_raw if self._task_phase == "WAIT_OPEN" else self._close_raw
            if message.data.startswith("OK ") and ("rx28=%d " % target) in message.data:
                self._motor_ack = True

    def _task_pose_callback(self, message):
        if self._distance_pose is None or message.header.stamp != self._distance_pose.header.stamp:
            self._distance_received = self._now()
        self._distance_pose = message

    def _task_distance_ok(self, now):
        pose = self._distance_pose
        if pose is None or now - self._distance_received > 0.60:
            return False
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec / 1e9
        target = float(self.get_parameter("grasp_distance").value)
        tolerance = float(self.get_parameter("grasp_distance_tolerance").value)
        return 0 <= now - stamp <= 2.0 and abs(pose.pose.position.z - target) <= tolerance

    def _start_task(self, request, response):
        if not self._enabled or self._task_phase not in ("IDLE", "DONE", "CANCELLED", "FAULT"):
            response.message = "task disabled or already running"
            return response
        if not (self._approach_client.service_is_ready() and self._guard_client.service_is_ready() and self._selector_client.service_is_ready()):
            response.message = "approach, guard or selector service unavailable"
            return response
        self._aligned_samples = 0
        self._task_started = self._now()
        self._stop_task_controls("OPEN")
        response.success = True
        response.message = "accepted: open -> approach -> stop -> grasp; lift disabled"
        self._publish_status("STARTED gripper-only task")
        return response

    def _cancel_task(self, request, response):
        self._stop_task_controls("CANCELLED")
        response.success = True
        response.message = "stopping base; gripper holds its last target"
        return response

    def _task_fault(self, reason):
        # Send both stop requests immediately; do not let a failed parameter
        # service prevent the independent velocity guard from stopping motion.
        if self._guard_client.service_is_ready():
            self._guard_client.call_async(SetBool.Request(data=False))
        if self._approach_client.service_is_ready():
            self._approach_client.call_async(SetBool.Request(data=False))
        self._task_queue = []
        self._task_future = None
        self._task_phase = "FAULT"
        self._publish_status("FAULT " + reason)

    def _task_timer(self):
        phase = self._task_phase
        if phase in ("IDLE", "DONE", "CANCELLED", "FAULT"):
            return
        now = self._now()
        if phase == "CONTROLS":
            if self._task_future is not None:
                if not self._task_future.done():
                    if now > self._control_deadline:
                        self._task_fault("control service timeout")
                    return
                try:
                    result = self._task_future.result()
                    okay = result is not None and (all(r.successful for r in result.results) if hasattr(result, "results") else result.success)
                    if not okay:
                        self._task_fault("control service rejected request")
                        return
                except Exception as exc:
                    self._task_fault(str(exc))
                    return
                self._task_future = None
            if self._task_queue:
                kind, value = self._task_queue.pop(0)
                if kind == "selector":
                    self._task_future = self._selector_client.call_async(SetParameters.Request(parameters=[Parameter("source_mode", value=value).to_parameter_msg()]))
                else:
                    client = self._guard_client if kind == "guard" else self._approach_client
                    self._task_future = client.call_async(SetBool.Request(data=value))
                self._control_deadline = now + 8.0
                return
            self._task_phase = self._task_next
            if self._task_phase == "APPROACHING":
                self._publish_status("APPROACHING waiting for fresh alignment and 20cm distance")
            if self._task_phase == "CANCELLED":
                self._publish_status("CANCELLED base stopped")
            return
        if phase in ("WAIT_OPEN", "WAIT_CLOSE") and not self._motor_ack and now > self._motor_ack_deadline:
            self._task_fault("motor command acknowledgment timeout")
            return
        if now - self._task_started > float(self.get_parameter("task_timeout").value):
            self._task_fault("task timed out")
            return
        if phase == "OPEN":
            self._motor_ack = False
            self._motor_ack_deadline = now + 3.0
            self._publish_raw(-1.0, float(self._open_raw), -1.0, -1.0, 1.0)
            self._deadline = now + float(self.get_parameter("open_wait").value)
            self._task_phase = "WAIT_OPEN"
            self._publish_status("OPENING before approach")
        elif phase == "WAIT_OPEN" and now >= self._deadline:
            self._aligned_samples = 0
            self._queue_task_controls([("selector", "APPROACH"), ("approach", True), ("guard", True)], "APPROACHING")
        elif phase == "APPROACHING":
            if self._aligned_samples >= 3 and now - self._alignment_received < 0.35 and self._tag_detected and self._task_distance_ok(now):
                self._stop_task_controls("CLOSE")
        elif phase == "CLOSE":
            # All three stop requests completed before commanding the gripper.
            if now - self._alignment_received > 0.35 or self._alignment_state != "ALIGNED" or not self._tag_detected or not self._task_distance_ok(now):
                self._task_fault("alignment/distance lost before closing")
                return
            self._motor_ack = False
            self._motor_ack_deadline = now + 3.0
            self._publish_raw(-1.0, float(self._close_raw), -1.0, -1.0, 1.0)
            self._deadline = now + self._close_wait
            self._task_phase = "WAIT_CLOSE"
            self._publish_status("CLOSING base stopped; lift disabled")
        elif phase == "WAIT_CLOSE" and now >= self._deadline:
            self._approach_client.call_async(SetBool.Request(data=False))
            self._task_phase = "DONE"
            self._state = SequenceState.DONE
            self._publish_status("DONE gripper target sent and close wait elapsed; start required for next cycle")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperSequenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
