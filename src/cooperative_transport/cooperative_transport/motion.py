"""Minimal motion types for manual-grasp transport; no gripper operations."""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class PlanarCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
