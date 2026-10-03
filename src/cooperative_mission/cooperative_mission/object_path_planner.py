"""Obstacle-free smooth pose-to-pose planning for cooperative transport."""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .hinged_formation import (
    FormationPath,
    HingeGeometry,
    object_path_to_robot_paths,
    path_curvatures,
)
from .motion import Pose2D, normalize_angle


CircleObstacle = Tuple[float, float, float]


def _bezier_pose(
    start: Pose2D, goal: Pose2D, handle: float, u: float
) -> Pose2D:
    """Evaluate a cubic Hermite-equivalent Bezier curve and its tangent yaw."""
    p0 = (start.x, start.y)
    p3 = (goal.x, goal.y)
    p1 = (start.x + handle * math.cos(start.yaw),
          start.y + handle * math.sin(start.yaw))
    p2 = (goal.x - handle * math.cos(goal.yaw),
          goal.y - handle * math.sin(goal.yaw))
    v = 1.0 - u
    x = v**3 * p0[0] + 3.0 * v**2 * u * p1[0] + 3.0 * v * u**2 * p2[0] + u**3 * p3[0]
    y = v**3 * p0[1] + 3.0 * v**2 * u * p1[1] + 3.0 * v * u**2 * p2[1] + u**3 * p3[1]
    dx = (
        3.0 * v**2 * (p1[0] - p0[0])
        + 6.0 * v * u * (p2[0] - p1[0])
        + 3.0 * u**2 * (p3[0] - p2[0])
    )
    dy = (
        3.0 * v**2 * (p1[1] - p0[1])
        + 6.0 * v * u * (p2[1] - p1[1])
        + 3.0 * u**2 * (p3[1] - p2[1])
    )
    if math.hypot(dx, dy) <= 1e-8:
        raise ValueError("candidate path has a stationary tangent")
    return Pose2D(x, y, math.atan2(dy, dx))


def _resample_curve(
    start: Pose2D, goal: Pose2D, handle: float, step: float, dense_count: int = 1200
) -> Tuple[Pose2D, ...]:
    dense = tuple(
        _bezier_pose(start, goal, handle, index / dense_count)
        for index in range(dense_count + 1)
    )
    cumulative = [0.0]
    for first, second in zip(dense, dense[1:]):
        cumulative.append(
            cumulative[-1] + math.hypot(second.x - first.x, second.y - first.y)
        )
    length = cumulative[-1]
    if length <= 1e-6:
        raise ValueError("start and goal positions must differ")
    if length > 2.0 * math.hypot(goal.x - start.x, goal.y - start.y):
        raise ValueError("candidate path is excessively long")

    count = max(2, int(math.ceil(length / step)))
    output: List[Pose2D] = []
    dense_index = 0
    for sample in range(count + 1):
        target = length * sample / count
        while dense_index + 1 < len(cumulative) - 1 and cumulative[dense_index + 1] < target:
            dense_index += 1
        span = cumulative[dense_index + 1] - cumulative[dense_index]
        ratio = 0.0 if span <= 1e-12 else (target - cumulative[dense_index]) / span
        first, second = dense[dense_index], dense[dense_index + 1]
        yaw_delta = normalize_angle(second.yaw - first.yaw)
        output.append(
            Pose2D(
                first.x + ratio * (second.x - first.x),
                first.y + ratio * (second.y - first.y),
                normalize_angle(first.yaw + ratio * yaw_delta),
            )
        )
    output[0] = Pose2D(start.x, start.y, start.yaw)
    output[-1] = Pose2D(goal.x, goal.y, goal.yaw)
    return tuple(output)


def _curve_cost(path: Sequence[Pose2D]) -> float:
    curvatures = path_curvatures(path)
    length = 0.0
    curvature_energy = 0.0
    for index, (first, second) in enumerate(zip(path, path[1:])):
        ds = math.hypot(second.x - first.x, second.y - first.y)
        length += ds
        curvature = 0.5 * (curvatures[index] + curvatures[index + 1])
        curvature_energy += curvature * curvature * ds
    # Prefer smooth curvature while keeping a modest preference for shorter paths.
    return curvature_energy + 0.05 * length


def _detour_candidates(
    start: Pose2D,
    goal: Pose2D,
    obstacles: Sequence[CircleObstacle],
    step: float,
) -> List[Tuple[Pose2D, ...]]:
    """Build smooth two-segment routes through points beside each obstacle."""
    output: List[Tuple[Pose2D, ...]] = []
    dx, dy = goal.x - start.x, goal.y - start.y
    distance = math.hypot(dx, dy)
    nx, ny = -dy / distance, dx / distance
    for ox, oy, radius in obstacles:
        # Try both sides of an obstacle in the direct start-to-goal corridor.
        for sign in (-1.0, 1.0):
            wx = ox + sign * nx * (radius + 0.72)
            wy = oy + sign * ny * (radius + 0.72)
            incoming = math.atan2(wy - start.y, wx - start.x)
            outgoing = math.atan2(goal.y - wy, goal.x - wx)
            delta = normalize_angle(outgoing - incoming)
            waypoint = Pose2D(wx, wy, normalize_angle(incoming + 0.5 * delta))
            for scale in (0.25, 0.4, 0.55, 0.7, 0.85):
                try:
                    first = _resample_curve(
                        start, waypoint,
                        math.hypot(wx - start.x, wy - start.y) * scale,
                        step,
                    )
                    second = _resample_curve(
                        waypoint, goal,
                        math.hypot(goal.x - wx, goal.y - wy) * scale,
                        step,
                    )
                    output.append(first[:-1] + second)
                except ValueError:
                    continue
    return output


def _circular_arc_candidate(
    start: Pose2D, goal: Pose2D, step: float
) -> Tuple[Pose2D, ...] | None:
    """Return the constant-curvature arc joining compatible endpoint poses."""
    delta = normalize_angle(goal.yaw - start.yaw)
    chord = math.hypot(goal.x - start.x, goal.y - start.y)
    if abs(delta) < 1e-4 or abs(abs(delta) - math.pi) < 1e-4 or chord <= 1e-8:
        return None
    curvature = 2.0 * math.sin(0.5 * delta) / chord
    if abs(curvature) <= 1e-8:
        return None
    radius = 1.0 / curvature
    cx = start.x - radius * math.sin(start.yaw)
    cy = start.y + radius * math.cos(start.yaw)
    count = max(2, int(math.ceil(abs(delta / curvature) / step)))
    poses = []
    for index in range(count + 1):
        ratio = index / count
        yaw = start.yaw + delta * ratio
        x = cx + radius * math.sin(yaw)
        y = cy - radius * math.cos(yaw)
        poses.append(Pose2D(x, y, normalize_angle(yaw)))
    endpoint_error = math.hypot(poses[-1].x - goal.x, poses[-1].y - goal.y)
    if endpoint_error > max(0.03, step):
        return None
    poses[0] = start
    poses[-1] = goal
    return tuple(poses)


def _formation_clear(
    formation: FormationPath,
    object_path: Sequence[Pose2D],
    obstacles: Sequence[CircleObstacle],
    bounds: Tuple[float, float, float, float] | None,
    robot_radius: float,
    safety_margin: float,
) -> bool:
    """Check conservative circular footprints for both robots and the box."""
    for index, obj in enumerate(object_path):
        centers = (
            (obj.x, obj.y, 0.10),  # box half diagonal, rounded outward
            (formation.leader[index].x, formation.leader[index].y, robot_radius),
            (formation.follower[index].x, formation.follower[index].y, robot_radius),
        )
        for x, y, footprint_radius in centers:
            if bounds is not None:
                xmin, xmax, ymin, ymax = bounds
                reach = footprint_radius + safety_margin
                if x - reach < xmin or x + reach > xmax or y - reach < ymin or y + reach > ymax:
                    return False
            for ox, oy, obstacle_radius in obstacles:
                clearance = footprint_radius + obstacle_radius + safety_margin
                if math.hypot(x - ox, y - oy) < clearance:
                    return False
    return True


def plan_object_path(
    start: Pose2D,
    goal: Pose2D,
    geometry: HingeGeometry,
    *,
    step: float = 0.05,
    curvature_margin: float = 0.8,
    lateral_tolerance: float = 0.005,
    obstacles: Sequence[CircleObstacle] = (),
    workspace_bounds: Tuple[float, float, float, float] | None = None,
    robot_radius: float = 0.32,
    safety_margin: float = 0.05,
) -> Tuple[Tuple[Pose2D, ...], FormationPath]:
    """Search smooth, collision-checked cubic routes and return the best route.

    The start and goal yaw define path tangents. Circular obstacle and workspace
    checks use conservative disk footprints for the box and each robot. Each
    candidate also observes hinge, curvature, and axle lateral-motion limits.
    """
    values = (start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw, step)
    if not all(math.isfinite(value) for value in values) or step <= 0.0:
        raise ValueError("poses must be finite and step positive")
    distance = math.hypot(goal.x - start.x, goal.y - start.y)
    if distance <= 1e-6:
        raise ValueError("start and goal positions must differ")

    if not math.isfinite(robot_radius) or robot_radius <= 0.0:
        raise ValueError("robot_radius must be finite and positive")
    if not math.isfinite(safety_margin) or safety_margin < 0.0:
        raise ValueError("safety_margin must be finite and non-negative")
    for obstacle in obstacles:
        if len(obstacle) != 3 or not all(math.isfinite(v) for v in obstacle) or obstacle[2] <= 0:
            raise ValueError("obstacles must be finite (x, y, positive_radius) circles")
    if workspace_bounds is not None:
        if len(workspace_bounds) != 4 or not all(math.isfinite(v) for v in workspace_bounds):
            raise ValueError("workspace_bounds must be finite (xmin, xmax, ymin, ymax)")
        xmin, xmax, ymin, ymax = workspace_bounds
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("workspace bounds must have positive width and height")

    candidates = []
    failures = []
    for scale in (0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 1.0, 1.15, 1.3):
        try:
            path = _resample_curve(start, goal, distance * scale, step)
            formation = object_path_to_robot_paths(
                path,
                geometry,
                curvature_margin=curvature_margin,
                lateral_tolerance=lateral_tolerance,
            )
            candidates.append((_curve_cost(path), path, formation))
        except ValueError as error:
            failures.append(str(error))
    candidate_paths = [candidate[1] for candidate in candidates]
    candidate_paths.extend(_detour_candidates(start, goal, obstacles, step))
    arc = _circular_arc_candidate(start, goal, step)
    if arc is not None:
        candidate_paths.append(arc)
    candidates = []
    for path in candidate_paths:
        try:
            formation = object_path_to_robot_paths(
                path, geometry, curvature_margin=curvature_margin,
                lateral_tolerance=lateral_tolerance,
            )
            if not _formation_clear(
                formation, path, obstacles, workspace_bounds,
                robot_radius, safety_margin,
            ):
                continue
            candidates.append((_curve_cost(path), path, formation))
        except ValueError as error:
            failures.append(str(error))
    if not candidates:
        if obstacles or workspace_bounds:
            reason = "all smooth candidates violate workspace/obstacle clearance"
        else:
            reason = failures[0] if failures else "no valid cubic candidate"
        raise ValueError("no feasible smooth path: %s" % reason)
    _, path, formation = min(candidates, key=lambda candidate: candidate[0])
    return path, formation


__all__ = ["plan_object_path"]
