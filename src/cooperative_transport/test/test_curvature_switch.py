"""Temporary curvature bypass keeps path generation and other checks intact."""
import math
import pytest

from cooperative_transport.hinged_formation import HingeGeometry, object_path_to_robot_paths
from cooperative_transport.motion import Pose2D


def tight_arc():
    return tuple(Pose2D(math.sin(i*.01), 1.-math.cos(i*.01), i*.01) for i in range(61))


def test_strict_default_rejects_tight_arc():
    with pytest.raises(ValueError, match='curvature'):
        object_path_to_robot_paths(tight_arc(), HingeGeometry())


def test_bypass_generates_path_above_curvature_and_hinge_limits():
    geometry = HingeGeometry()
    result = object_path_to_robot_paths(tight_arc(), geometry, validate_curvature=False)
    assert result.max_abs_curvature > geometry.curvature_limit(.8)
    assert max(abs(a) for a in result.leader_hinge_angles) > geometry.hinge_limit*.8
    assert len(result.follower) == len(tight_arc())
    assert all(math.isfinite(p.x+p.y+p.yaw) for p in result.follower)


def test_bypass_still_rejects_lateral_path():
    with pytest.raises(ValueError, match='lateral motion'):
        object_path_to_robot_paths((Pose2D(0.,0.,0.),Pose2D(0.,.1,0.)),
                                   HingeGeometry(), validate_curvature=False)
