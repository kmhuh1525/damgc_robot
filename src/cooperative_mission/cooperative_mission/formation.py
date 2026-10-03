"""Rigid-grasp formation geometry for Leader-to-Follower path conversion.

All transforms use ``T_AB`` notation: a transform stored as :class:`Pose2D`
maps coordinates from frame B into frame A. The four grasp transforms are
calibrated from the robot CAD and the chosen contact points on the box.

This module derives the Follower reference pose and the twist required to keep
the grasp geometry rigid. It deliberately reports lateral velocity: a
differential-drive robot cannot realize a rigid formation command when that
component is non-zero, so the caller must stop or request a feasible path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List

from .motion import Pose2D, is_finite, normalize_angle


@dataclass(frozen=True)
class GraspGeometry:
    """Calibrated planar transforms between bases, grippers, and box.

    ``leader_base_to_gripper`` is T_L_GL, ``object_to_leader_gripper`` is
    T_O_GL, ``object_to_follower_gripper`` is T_O_GF, and
    ``follower_base_to_gripper`` is T_F_GF. Box dimensions and grasp-point
    offsets are used when constructing the two object-to-gripper transforms;
    robot CAD supplies the base-to-gripper transforms.
    """

    box_length: float
    box_width: float
    leader_base_to_gripper: Pose2D
    object_to_leader_gripper: Pose2D
    object_to_follower_gripper: Pose2D
    follower_base_to_gripper: Pose2D

    def validate(self) -> None:
        if not is_finite(self.box_length, self.box_width):
            raise ValueError("box dimensions must be finite")
        if self.box_length <= 0.0 or self.box_width <= 0.0:
            raise ValueError("box dimensions must be positive")
        for name in (
            "leader_base_to_gripper",
            "object_to_leader_gripper",
            "object_to_follower_gripper",
            "follower_base_to_gripper",
        ):
            pose = getattr(self, name)
            if not is_finite(pose.x, pose.y, pose.yaw):
                raise ValueError("%s must contain finite values" % name)


@dataclass(frozen=True)
class FollowerTwist:
    """Follower body-frame velocity and rigid-formation feasibility."""

    linear_x: float
    linear_y_required: float
    angular_z: float

    def is_differential_drive_feasible(self, lateral_tolerance: float = 0.01) -> bool:
        return (
            math.isfinite(lateral_tolerance)
            and lateral_tolerance >= 0.0
            and abs(self.linear_y_required) <= lateral_tolerance
        )


def compose_pose(a_from_b: Pose2D, b_from_c: Pose2D) -> Pose2D:
    """Compose T_AB and T_BC to return T_AC."""
    c = math.cos(a_from_b.yaw)
    s = math.sin(a_from_b.yaw)
    return Pose2D(
        a_from_b.x + c * b_from_c.x - s * b_from_c.y,
        a_from_b.y + s * b_from_c.x + c * b_from_c.y,
        normalize_angle(a_from_b.yaw + b_from_c.yaw),
    )


def inverse_pose(a_from_b: Pose2D) -> Pose2D:
    """Invert T_AB to return T_BA."""
    c = math.cos(a_from_b.yaw)
    s = math.sin(a_from_b.yaw)
    return Pose2D(
        -c * a_from_b.x - s * a_from_b.y,
        s * a_from_b.x - c * a_from_b.y,
        normalize_angle(-a_from_b.yaw),
    )


def leader_pose_to_follower(
    leader_world: Pose2D, geometry: GraspGeometry
) -> Pose2D:
    """Derive the Follower world pose from one Leader world pose.

    The chain is Leader base -> Leader gripper -> box -> Follower gripper ->
    Follower base. Once both grippers hold rigid contact points, this transform
    is constant and every Leader path pose maps directly to a Follower pose.
    """
    geometry.validate()
    world_from_object = compose_pose(
        compose_pose(leader_world, geometry.leader_base_to_gripper),
        inverse_pose(geometry.object_to_leader_gripper),
    )
    world_from_follower_gripper = compose_pose(
        world_from_object,
        geometry.object_to_follower_gripper,
    )
    return compose_pose(
        world_from_follower_gripper,
        inverse_pose(geometry.follower_base_to_gripper),
    )


def transform_leader_path(
    leader_path: Iterable[Pose2D], geometry: GraspGeometry
) -> List[Pose2D]:
    """Map each Leader path pose to its rigid-formation Follower pose."""
    return [leader_pose_to_follower(pose, geometry) for pose in leader_path]


def leader_twist_to_follower(
    leader_world: Pose2D,
    leader_linear_x: float,
    leader_angular_z: float,
    geometry: GraspGeometry,
) -> FollowerTwist:
    """Transform a Leader body twist to the Follower base frame.

    The velocity at the Follower base includes the rotational term
    ``omega x relative_position``. This is essential on curved paths; merely
    changing the sign of Leader ``linear.x`` omits that term.
    """
    geometry.validate()
    if not is_finite(leader_world.x, leader_world.y, leader_world.yaw,
                     leader_linear_x, leader_angular_z):
        raise ValueError("Leader pose and twist must be finite")

    follower_world = leader_pose_to_follower(leader_world, geometry)
    leader_from_follower = compose_pose(inverse_pose(leader_world), follower_world)
    c_l = math.cos(leader_world.yaw)
    s_l = math.sin(leader_world.yaw)
    leader_vx_world = c_l * leader_linear_x
    leader_vy_world = s_l * leader_linear_x
    rx_world = c_l * leader_from_follower.x - s_l * leader_from_follower.y
    ry_world = s_l * leader_from_follower.x + c_l * leader_from_follower.y

    follower_vx_world = leader_vx_world - leader_angular_z * ry_world
    follower_vy_world = leader_vy_world + leader_angular_z * rx_world
    c_f = math.cos(follower_world.yaw)
    s_f = math.sin(follower_world.yaw)
    follower_vx = c_f * follower_vx_world + s_f * follower_vy_world
    follower_vy = -s_f * follower_vx_world + c_f * follower_vy_world
    return FollowerTwist(
        linear_x=follower_vx,
        linear_y_required=follower_vy,
        angular_z=leader_angular_z,
    )


__all__ = [
    "FollowerTwist",
    "GraspGeometry",
    "compose_pose",
    "inverse_pose",
    "leader_pose_to_follower",
    "leader_twist_to_follower",
    "transform_leader_path",
]
