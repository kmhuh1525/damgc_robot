"""Golden paths produced by the real leader commit, independent of local math."""
import json
from pathlib import Path

import pytest

from cooperative_transport.hinged_formation import HingeGeometry, leader_path_to_object_path
from cooperative_transport.motion import Pose2D

CASES = json.loads((Path(__file__).parent/'fixtures/leader_d812a04_paths.json').read_text())['cases']


@pytest.mark.parametrize('case', CASES, ids=lambda case: case['name'])
def test_conversion_matches_real_leader_commit(case):
    points = tuple(Pose2D(*p) for p in case['body']['leader'])
    obj, result = leader_path_to_object_path(points, HingeGeometry())
    for actual, expected in zip(result.follower,case['body']['follower']):
        assert (actual.x,actual.y,actual.yaw) == pytest.approx(expected,abs=1e-12)
    for actual, expected in zip(obj,case['object']):
        assert (actual.x,actual.y,actual.yaw) == pytest.approx(expected,abs=1e-12)
    assert len(result.follower) == len(case['body']['follower'])


def test_iteration_compatibility_argument_does_not_change_direct_solution():
    points = tuple(Pose2D(*p) for p in CASES[1]['body']['leader'])
    assert leader_path_to_object_path(points,HingeGeometry(),iterations=1) == \
        leader_path_to_object_path(points,HingeGeometry(),iterations=16)
