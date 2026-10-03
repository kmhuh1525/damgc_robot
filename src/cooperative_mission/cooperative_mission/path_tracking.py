"""Pure-pursuit tracking math for a differential-drive robot path preview."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .hinged_formation import drive_direction, path_curvatures
from .motion import PlanarCommand, Pose2D


@dataclass(frozen=True)
class TrackingResult:
    command: PlanarCommand
    progress_index: int
    cross_track_error: float
    direction: str
    reached_goal: bool
    detail: str


def compute_tracking_command(
    path: Sequence[Pose2D],
    robot: Pose2D,
    progress_index: int,
    *,
    lookahead_distance: float = 0.20,
    max_path_error: float = 0.40,
    goal_tolerance: float = 0.05,
    max_linear_speed: float = 0.05,
    max_angular_speed: float = 0.20,
    lateral_acceleration: float = 0.08,
) -> TrackingResult:
    """Compute a bounded pure-pursuit Twist, including reverse travel.

    The sign of linear speed comes from path displacement projected onto the
    robot heading. Angular speed uses signed speed times path curvature, which
    keeps the steering sign correct when the robot travels backward.
    """
    values = (
        lookahead_distance,
        max_path_error,
        goal_tolerance,
        max_linear_speed,
        max_angular_speed,
        lateral_acceleration,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("tracking limits must be finite and positive")
    if len(path) < 2:
        return TrackingResult(PlanarCommand(), 0, math.inf, "UNKNOWN", False, "path_not_ready")

    first_index = max(0, min(progress_index, len(path) - 1) - 10)
    nearest = min(
        range(first_index, len(path)),
        key=lambda index: math.hypot(path[index].x - robot.x, path[index].y - robot.y),
    )
    error = math.hypot(path[nearest].x - robot.x, path[nearest].y - robot.y)
    progress = max(progress_index, nearest)
    if error > max_path_error:
        return TrackingResult(PlanarCommand(), progress, error, "UNKNOWN", False, "path_error_limit")
    if math.hypot(path[-1].x - robot.x, path[-1].y - robot.y) <= goal_tolerance:
        return TrackingResult(PlanarCommand(), len(path) - 1, error, "STOPPED", True, "goal_reached")

    travelled = 0.0
    target_index = nearest
    for index in range(nearest + 1, len(path)):
        travelled += math.hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y)
        target_index = index
        if travelled >= lookahead_distance:
            break
    target = path[target_index]

    direction = drive_direction(path)
    if direction in ("MIXED", "LATERAL_ORIENTATION_MISMATCH"):
        return TrackingResult(PlanarCommand(), progress, error, direction, False, "path_direction_not_constant")
    direction_sign = 1.0 if direction == "FORWARD" else -1.0

    dx, dy = target.x - robot.x, target.y - robot.y
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    body_x = c * dx + s * dy
    body_y = -s * dx + c * dy
    distance_sq = max(dx * dx + dy * dy, lookahead_distance * lookahead_distance)
    pursuit_curvature = 2.0 * body_y / distance_sq

    path_curvature = max(abs(value) for value in path_curvatures(path)[max(0, nearest - 1):nearest + 2])
    speed_limit = max_linear_speed
    if path_curvature > 1e-6:
        speed_limit = min(speed_limit, math.sqrt(lateral_acceleration / path_curvature))
    signed_speed = direction_sign * speed_limit
    angular = signed_speed * pursuit_curvature
    angular = max(-max_angular_speed, min(max_angular_speed, angular))
    return TrackingResult(
        PlanarCommand(signed_speed, angular),
        progress,
        error,
        direction,
        False,
        "tracking",
    )


__all__ = ["TrackingResult", "compute_tracking_command"]
