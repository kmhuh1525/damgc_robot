from collections import Counter
from cooperative_manual.opening_prepare import classify
from cooperative_manual.follower_prepare import REQUIRED


def test_opening_reuses_complete_or_idle_drive_without_duplicate_hardware():
    assert classify(Counter())==10
    assert classify(Counter(REQUIRED))==0
    assert classify(Counter(REQUIRED[:-1]))==12
    for extra in ('manual_curve','gripper_sequence','mission_executor','transport_peer','stm32_bridge'):
        assert classify(Counter(REQUIRED[:-1]+(extra,)))==11
