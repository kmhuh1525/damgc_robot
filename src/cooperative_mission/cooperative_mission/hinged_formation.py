"""Kinematics and curvature-limited paths for the passive-hinge formation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .motion import Pose2D, normalize_angle


@dataclass(frozen=True)
class HingeGeometry:
    """Planar geometry, measured from each robot axle to the held box.

    ``axle_to_hinge`` and ``hinge_to_contact`` are collinear in the neutral
    pose. The passive yaw hinge lets the robot body rotate relative to the arm
    and held object by ``hinge_limit``.
    """

    axle_to_hinge: float = 0.125
    hinge_to_contact: float = 0.1325
    object_center_to_contact: float = 0.0525
    hinge_limit: float = math.radians(15.0)

    def validate(self) -> None:
        values = (
            self.axle_to_hinge,
            self.hinge_to_contact,
            self.object_center_to_contact,
            self.hinge_limit,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("hinge geometry must be finite")
        if min(values[:3]) <= 0.0 or not 0.0 < self.hinge_limit < math.pi / 2:
            raise ValueError("hinge lengths and angle limit must be positive")

    @property
    def longitudinal_offset(self) -> float:
        return self.hinge_to_contact + self.object_center_to_contact

    def curvature_limit(self, margin: float = 1.0) -> float:
        """Maximum constant path curvature allowed by the hinge angle.

        ``margin`` scales the physical angle limit down for a planning reserve.
        """
        if not math.isfinite(margin) or not 0.0 < margin <= 1.0:
            raise ValueError("margin must be in (0, 1]")
        angle = self.hinge_limit * margin
        numerator = math.sin(angle)
        denominator = (
            self.longitudinal_offset * math.cos(angle) + self.axle_to_hinge
        )
        return numerator / denominator


@dataclass(frozen=True)
class FormationPath:
    leader: Tuple[Pose2D, ...]
    follower: Tuple[Pose2D, ...]
    leader_hinge_angles: Tuple[float, ...]
    follower_hinge_angles: Tuple[float, ...]
    max_abs_curvature: float
    max_lateral_ratio: float


def smooth_turn_path(
    *,
    turn_angle: float = math.pi / 2.0,
    curvature: float = 0.25,
    curvature_ramp_length: float = 1.0,
    step: float = 0.05,
    straight_before: float = 0.6,
) -> Tuple[Pose2D, ...]:
    """Create a straight-to-turn-to-straight object path with continuous curvature."""
    scalars = (turn_angle, curvature, curvature_ramp_length, step, straight_before)
    if not all(math.isfinite(value) for value in scalars):
        raise ValueError("path parameters must be finite")
    if turn_angle == 0.0 or curvature <= 0.0 or curvature_ramp_length <= 0.0:
        raise ValueError("turn and curvature parameters must be non-zero")
    if step <= 0.0 or straight_before < 0.0:
        raise ValueError("step must be positive and straight_before non-negative")

    direction = math.copysign(1.0, turn_angle)
    angle = abs(turn_angle)
    ramp_angle = curvature * curvature_ramp_length
    if angle <= ramp_angle:
        ramp_length = math.sqrt(angle / curvature)
        ramp_angle = angle
    else:
        ramp_length = curvature_ramp_length
    plateau_length = max(0.0, (angle - ramp_angle) / curvature)

    def advance(length: float, curvature_at) -> None:
        nonlocal x, y, yaw
        count = max(1, int(math.ceil(length / step)))
        ds = length / count
        for index in range(count):
            k0 = direction * curvature_at(index / count)
            k1 = direction * curvature_at((index + 1) / count)
            km = 0.5 * (k0 + k1)
            dyaw = km * ds
            yaw_mid = yaw + 0.5 * dyaw
            x += ds * math.cos(yaw_mid)
            y += ds * math.sin(yaw_mid)
            yaw = normalize_angle(yaw + dyaw)
            points.append(Pose2D(x, y, yaw))

    x, y, yaw = 0.0, 0.0, 0.0
    points: List[Pose2D] = [Pose2D(-straight_before, 0.0, 0.0)]
    # Include the straight segment with consistent spacing.
    count = max(1, int(math.ceil(straight_before / step))) if straight_before else 0
    for index in range(1, count + 1):
        x = -straight_before + straight_before * index / count
        points.append(Pose2D(x, 0.0, 0.0))

    advance(ramp_length, lambda u: curvature * u)
    advance(plateau_length, lambda _u: curvature)
    advance(ramp_length, lambda u: curvature * (1.0 - u))
    return tuple(points)


def path_curvatures(path: Sequence[Pose2D]) -> Tuple[float, ...]:
    """Estimate signed curvature at each path pose from adjacent samples."""
    if len(path) < 2:
        raise ValueError("path must contain at least two poses")
    values: List[float] = []
    for index, pose in enumerate(path):
        if index == 0:
            first, second = path[0], path[1]
        elif index == len(path) - 1:
            first, second = path[-2], path[-1]
        else:
            first, second = path[index - 1], path[index + 1]
        distance = math.hypot(second.x - first.x, second.y - first.y)
        if distance <= 1e-9:
            values.append(0.0)
            continue
        delta_yaw = normalize_angle(second.yaw - first.yaw)
        values.append(delta_yaw / distance)
    return tuple(values)


def _equilibrium_hinge_angle(curvature: float, geometry: HingeGeometry) -> float:
    """Return the steady low-speed hinge angle for constant signed curvature."""
    offset = geometry.longitudinal_offset
    lever = geometry.axle_to_hinge
    phase = math.atan(curvature * offset)
    argument = curvature * lever / math.sqrt(1.0 + (curvature * offset) ** 2)
    if abs(argument) > 1.0:
        raise ValueError("path curvature has no hinge-angle solution")
    return phase + math.asin(argument)


def leader_path_to_object_path(
    leader_path: Sequence[Pose2D],
    geometry: HingeGeometry,
    *,
    curvature_margin: float = 0.8,
    lateral_tolerance: float = 0.01,
    iterations: int = 16,
) -> Tuple[Tuple[Pose2D, ...], "FormationPath"]:
    """Invert a leader axle path into the shared object path.

    The hinge angle depends on object-path curvature, so the conversion
    iterates between the measured leader curvature and reconstructed object
    curvature. A forward conversion is then checked against the source path.
    """
    if len(leader_path) < 3:
        raise ValueError("leader path needs at least three poses to estimate curvature")
    if iterations < 1:
        raise ValueError("iterations must be positive")
    geometry.validate()
    def smooth(values: Sequence[float]) -> Tuple[float, ...]:
        radius = min(3, max(1, len(values) // 30))
        return tuple(
            sum(values[max(0, index - radius):min(len(values), index + radius + 1)])
            / len(values[max(0, index - radius):min(len(values), index + radius + 1)])
            for index in range(len(values))
        )

    axle_curvatures = smooth(path_curvatures(leader_path))
    object_curvatures = tuple(axle_curvatures)
    object_path: Tuple[Pose2D, ...] = ()
    for _ in range(iterations):
        reconstructed = []
        for base, curvature in zip(leader_path, object_curvatures):
            hinge_angle = _equilibrium_hinge_angle(curvature, geometry)
            object_yaw = normalize_angle(base.yaw + hinge_angle)
            reconstructed.append(
                Pose2D(
                    base.x
                    + geometry.axle_to_hinge * math.cos(base.yaw)
                    + geometry.longitudinal_offset * math.cos(object_yaw),
                    base.y
                    + geometry.axle_to_hinge * math.sin(base.yaw)
                    + geometry.longitudinal_offset * math.sin(object_yaw),
                    object_yaw,
                )
            )
        object_path = tuple(reconstructed)
        updated = smooth(path_curvatures(object_path))
        # Damping prevents noisy sampled path curvature from amplifying through
        # the offset hinge geometry during the fixed-point inversion.
        object_curvatures = tuple(
            0.65 * previous + 0.35 * current
            for previous, current in zip(object_curvatures, updated)
        )

    formation = object_path_to_robot_paths(
        object_path,
        geometry,
        curvature_margin=curvature_margin,
        lateral_tolerance=lateral_tolerance,
    )
    max_position_error = max(
        math.hypot(actual.x - expected.x, actual.y - expected.y)
        for actual, expected in zip(formation.leader, leader_path)
    )
    max_yaw_error = max(
        abs(normalize_angle(actual.yaw - expected.yaw))
        for actual, expected in zip(formation.leader, leader_path)
    )
    if max_position_error > 0.02 or max_yaw_error > math.radians(2.0):
        raise ValueError(
            "leader/object path round-trip mismatch position=%.3f m yaw=%.2f deg"
            % (max_position_error, math.degrees(max_yaw_error))
        )
    return object_path, formation


def drive_direction(path: Sequence[Pose2D]) -> str:
    """Classify whether the robot body follows a path forward or in reverse."""
    if len(path) < 2:
        raise ValueError("drive direction needs at least two poses")
    directions = set()
    for first, second in zip(path, path[1:]):
        dx, dy = second.x - first.x, second.y - first.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            continue
        yaw_mid = first.yaw + 0.5 * normalize_angle(second.yaw - first.yaw)
        projection = math.cos(yaw_mid) * dx + math.sin(yaw_mid) * dy
        if abs(projection) / distance < 0.05:
            return "LATERAL_ORIENTATION_MISMATCH"
        directions.add("FORWARD" if projection > 0.0 else "REVERSE")
    if not directions:
        raise ValueError("path contains no translational motion")
    return next(iter(directions)) if len(directions) == 1 else "MIXED"


def _max_lateral_ratio(path: Sequence[Pose2D]) -> float:
    """Measure segment motion sideways to each robot's interpolated heading."""
    maximum = 0.0
    for previous, current in zip(path, path[1:]):
        dx = current.x - previous.x
        dy = current.y - previous.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            continue
        yaw_mid = previous.yaw + 0.5 * normalize_angle(current.yaw - previous.yaw)
        lateral = -math.sin(yaw_mid) * dx + math.cos(yaw_mid) * dy
        maximum = max(maximum, abs(lateral) / distance)
    return maximum


def object_path_to_robot_paths(
    object_path: Sequence[Pose2D],
    geometry: HingeGeometry,
    *,
    curvature_margin: float = 0.8,
    lateral_tolerance: float = 0.01,
) -> FormationPath:
    """Convert an object-center path into both axle paths with passive hinge angles.

    The curvature reserve defaults to 80% of the measured hinge limit. Hinge
    angles use a quasi-static constant-curvature model; each generated axle
    path is then checked for lateral displacement at its finite sample spacing.
    """
    geometry.validate()
    if len(object_path) < 2:
        raise ValueError("object path must contain at least two poses")
    if not math.isfinite(lateral_tolerance) or lateral_tolerance < 0.0:
        raise ValueError("lateral_tolerance must be finite and non-negative")
    curvatures = path_curvatures(object_path)
    max_curvature = max(abs(value) for value in curvatures)
    limit = geometry.curvature_limit(curvature_margin)
    if max_curvature > limit + 1e-6:
        raise ValueError(
            "path curvature %.4f 1/m exceeds cooperative limit %.4f 1/m"
            % (max_curvature, limit)
        )

    leader: List[Pose2D] = []
    follower: List[Pose2D] = []
    leader_angles: List[float] = []
    follower_angles: List[float] = []
    leader_hinge_angles = tuple(
        _equilibrium_hinge_angle(curvature, geometry) for curvature in curvatures
    )
    follower_hinge_angles = tuple(-angle for angle in leader_hinge_angles)
    allowed_hinge = geometry.hinge_limit * curvature_margin
    max_hinge = max(abs(angle) for angle in leader_hinge_angles)
    if max_hinge > allowed_hinge + 1e-5:
        raise ValueError(
            "required passive hinge angle %.2f deg exceeds reserved limit %.2f deg"
            % (
                math.degrees(max_hinge),
                math.degrees(allowed_hinge),
            )
        )

    for obj, q, q_follower in zip(
        object_path, leader_hinge_angles, follower_hinge_angles
    ):
        if abs(q) > allowed_hinge + 1e-5:
            raise ValueError("required passive hinge angle exceeds reserved limit")
        c = math.cos(obj.yaw)
        s = math.sin(obj.yaw)

        # Robot bases are found by walking backward from each box contact:
        # contact -> arm end -> hinge -> axle.
        leader_yaw = normalize_angle(obj.yaw - q)
        leader_x = (
            obj.x
            - geometry.longitudinal_offset * c
            - geometry.axle_to_hinge * math.cos(leader_yaw)
        )
        leader_y = (
            obj.y
            - geometry.longitudinal_offset * s
            - geometry.axle_to_hinge * math.sin(leader_yaw)
        )

        follower_yaw = normalize_angle(obj.yaw + math.pi + q)
        follower_x = (
            obj.x
            + geometry.longitudinal_offset * c
            - geometry.axle_to_hinge * math.cos(follower_yaw)
        )
        follower_y = (
            obj.y
            + geometry.longitudinal_offset * s
            - geometry.axle_to_hinge * math.sin(follower_yaw)
        )
        leader.append(Pose2D(leader_x, leader_y, leader_yaw))
        follower.append(Pose2D(follower_x, follower_y, follower_yaw))
        leader_angles.append(q)
        follower_angles.append(q_follower)

    lateral_ratio = max(
        _max_lateral_ratio(leader), _max_lateral_ratio(follower)
    )
    if lateral_ratio > lateral_tolerance:
        raise ValueError(
            "discrete robot path needs lateral motion ratio %.5f (limit %.5f)"
            % (lateral_ratio, lateral_tolerance)
        )

    return FormationPath(
        leader=tuple(leader),
        follower=tuple(follower),
        leader_hinge_angles=tuple(leader_angles),
        follower_hinge_angles=tuple(follower_angles),
        max_abs_curvature=max_curvature,
        max_lateral_ratio=lateral_ratio,
    )


__all__ = [
    "FormationPath",
    "HingeGeometry",
    "object_path_to_robot_paths",
    "leader_path_to_object_path",
    "drive_direction",
    "path_curvatures",
    "smooth_turn_path",
]
