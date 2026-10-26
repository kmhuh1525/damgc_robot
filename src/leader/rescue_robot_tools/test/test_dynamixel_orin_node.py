"""Safety and compatibility tests for raw Dynamixel command routing."""

from pathlib import Path
from types import SimpleNamespace

from std_msgs.msg import Float64MultiArray

from dynamixel_orin import DynamixelCommunicationError
from dynamixel_orin_node import DynamixelOrinNode


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def make_harness():
    calls = []
    statuses = []
    return SimpleNamespace(
        controller=SimpleNamespace(
            set_torque=lambda device_id, enabled: calls.append(
                ("torque", device_id, enabled)
            ),
            set_position=lambda device_id, position, minimum, maximum: calls.append(
                ("position", device_id, position, minimum, maximum)
            ),
            set_rx64_speed=lambda speed: calls.append(("rx64_speed", speed)),
        ),
        profile={
            "rx64_id": 33,
            "rx64_min": 260,
            "rx64_max": 670,
            "rx28_id": 2,
            "rx28_min": 1,
            "rx28_max": 1021,
        },
        get_parameter=lambda _: SimpleNamespace(value=True),
        calls=calls,
        publish_status=statuses.append,
        statuses=statuses,
    )


def test_targeted_gripper_command_does_not_enable_rx64():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 1000.0, -1.0, 1.0])
    )

    assert harness.calls == [("torque", 2, True), ("position", 2, 1000, 1, 1021)]


def test_targeted_lift_command_does_not_touch_rx28():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[520.0, -1.0, 1.0, -1.0])
    )

    assert harness.calls == [("torque", 33, True), ("position", 33, 520, 260, 670)]


def test_legacy_three_field_command_remains_compatible():
    harness = make_harness()

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 450.0, 1.0])
    )

    assert ("torque", 33, True) in harness.calls
    assert ("torque", 2, True) in harness.calls
    assert ("position", 2, 450, 1, 1021) in harness.calls


def test_semantic_open_and_close_use_rx28_targeted_writes():
    harness = make_harness()

    DynamixelOrinNode.gripper_command_callback(
        harness, SimpleNamespace(data="open")
    )
    DynamixelOrinNode.gripper_command_callback(
        harness, SimpleNamespace(data="close")
    )

    assert harness.calls == [
        ("torque", 2, True),
        ("position", 2, 1021, 1, 1021),
        ("torque", 2, True),
        ("position", 2, 1, 1, 1021),
    ]
    assert harness.statuses == [
        "OK RX28 position=1021 command=open",
        "OK RX28 position=1 command=close",
    ]


def test_sdk_failure_is_published_as_error_status():
    harness = make_harness()
    harness.controller.set_torque = lambda device_id, enabled: (_ for _ in ()).throw(
        DynamixelCommunicationError("RX28 torque: tx failure")
    )

    DynamixelOrinNode.command_callback(
        harness, Float64MultiArray(data=[-1.0, 1000.0, -1.0, 1.0])
    )

    assert harness.statuses == ["ERROR RX28 torque: tx failure"]


def test_disabled_startup_pose_and_torque_write_nothing():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=False,
        startup_torque=False,
        startup_rx64=600,
        startup_rx28=500,
    )

    assert harness.calls == []
    assert harness.statuses == ["STARTUP pose disabled torque=0"]


def test_rx64_speed_is_applied_without_torque_or_position_write():
    harness = make_harness()

    DynamixelOrinNode._apply_rx64_speed(harness, 10)

    assert harness.calls == [("rx64_speed", 10)]
    assert harness.statuses == ["RX64_SPEED raw=10"]


def test_leader_compatible_startup_pose_remains_enabled():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=True,
        startup_torque=True,
        startup_rx64=600,
        startup_rx28=500,
    )

    assert harness.calls == [
        ("torque", 33, True),
        ("torque", 2, True),
        ("position", 33, 600, 260, 670),
        ("position", 2, 500, 1, 1021),
    ]


def test_startup_pose_disabled_does_not_validate_unused_positions():
    harness = make_harness()

    DynamixelOrinNode._apply_startup_configuration(
        harness,
        startup_pose_enabled=False,
        startup_torque=False,
        startup_rx64=-1,
        startup_rx28=-1,
    )

    assert harness.calls == []


def test_shared_startup_defaults_remain_leader_compatible_and_typed():
    source = (PACKAGE_ROOT / "launch/dynamixel_orin.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("robot", default_value="leader")' in source
    assert 'DeclareLaunchArgument("rx64_speed", default_value="10")' in source
    assert 'DeclareLaunchArgument("startup_torque", default_value="true")' in source
    assert (
        'DeclareLaunchArgument("startup_pose_enabled", default_value="true")'
        in source
    )
    assert 'LaunchConfiguration("startup_torque"), value_type=bool' in source
    assert 'LaunchConfiguration("startup_pose_enabled"), value_type=bool' in source
    assert 'LaunchConfiguration("rx64_speed"), value_type=int' in source

    node_source = (PACKAGE_ROOT / "scripts/dynamixel_orin_node.py").read_text(
        encoding="utf-8"
    )
    assert 'self.declare_parameter("rx64_speed", 10)' in node_source
    assert "self._apply_rx64_speed(rx64_speed)" in node_source


def test_disabled_lift_ignores_raw_position_and_torque_enable():
    harness = make_harness()
    harness.get_parameter = lambda _: SimpleNamespace(value=False)
    DynamixelOrinNode.command_callback(harness, Float64MultiArray(data=[45.0, -1.0, 1.0, -1.0]))
    assert not harness.calls


def test_disabled_lift_legacy_command_only_touches_gripper():
    harness = make_harness()
    harness.get_parameter = lambda _: SimpleNamespace(value=False)
    DynamixelOrinNode.command_callback(harness, Float64MultiArray(data=[45.0, 331.0, 1.0]))
    assert harness.calls == [("torque", 2, True), ("position", 2, 331, 1, 1021)]


def test_disabled_lift_rejects_semantic_motion():
    harness = make_harness()
    harness.get_parameter = lambda _: SimpleNamespace(value=False)
    DynamixelOrinNode.gripper_command_callback(harness, SimpleNamespace(data="rx64_high"))
    assert not harness.calls
    assert harness.statuses == ["IGNORED lift disabled"]
