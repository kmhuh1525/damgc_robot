"""Small rclpy helpers shared by the two mission nodes."""

from __future__ import annotations

import math
from typing import Callable, Optional

from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from .actions import Action
from .motion import PlanarCommand

# Same profile as the existing selector/guard command topics.
COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# Orin<->Orin mission messages: keep a short reliable backlog.
LINK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# Latched state topics, compatible with leader_cooperation's /mission/state.
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

ResultCallback = Callable[[int, bool, str, float], None]


def to_twist(command: PlanarCommand) -> Twist:
    message = Twist()
    message.linear.x = float(command.linear_x)
    message.angular.z = float(command.angular_z)
    return message


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def call_set_bool(
    client,
    action: Action,
    now_fn: Callable[[], float],
    on_result: ResultCallback,
    logger,
) -> None:
    """Call a ``std_srvs/SetBool`` service asynchronously and report the result."""
    if not client.service_is_ready():
        on_result(action.action_id, False, f"service {client.srv_name} unavailable", now_fn())
        return
    request = SetBool.Request()
    request.data = bool(action.value)
    future = client.call_async(request)

    def _done(done_future) -> None:
        success, detail = _future_result(done_future)
        if success:
            response = done_future.result()
            success = bool(getattr(response, "success", False))
            detail = str(getattr(response, "message", ""))
        if not success:
            logger.error(f"{client.srv_name} data={action.value} failed: {detail}")
        on_result(action.action_id, success, detail, now_fn())

    future.add_done_callback(_done)


def _future_result(future) -> tuple:
    try:
        exception: Optional[BaseException] = future.exception()
    except Exception as error:  # pragma: no cover - defensive
        return False, str(error)
    if exception is not None:
        return False, str(exception)
    if future.result() is None:
        return False, "no response"
    return True, ""
