"""Protocol 1.0 SDK write and communication-result tests."""

import pytest

import dynamixel_orin as module


@pytest.fixture
def sdk_controller(monkeypatch):
    calls = []

    class FakePort:
        def __init__(self, port):
            calls.append(("port", port))

        def openPort(self):
            calls.append(("open",))
            return True

        def setBaudRate(self, baudrate):
            calls.append(("baud", baudrate))
            return True

        def closePort(self):
            calls.append(("close",))

    class FakePacket:
        def __init__(self, protocol):
            calls.append(("protocol", protocol))
            self.tx_result = 0
            self.packet_error = 0

        def write1ByteTxRx(self, port, device_id, address, data):
            calls.append(("write1", device_id, address, data))
            return self.tx_result, self.packet_error

        def write2ByteTxRx(self, port, device_id, address, data):
            calls.append(("write2", device_id, address, data))
            return self.tx_result, self.packet_error

        def getTxRxResult(self, result):
            return f"tx={result}"

        def getRxPacketError(self, error):
            return f"packet={error}"

    monkeypatch.setattr(module, "PortHandler", FakePort)
    monkeypatch.setattr(module, "PacketHandler", FakePacket)
    monkeypatch.setattr(module, "COMM_SUCCESS", 0)
    controller = module.DynamixelOrin("/dev/ttyUSB0", module.get_profile("leader"))
    return controller, calls


def test_protocol_1_sdk_writes_targeted_rx28_only(sdk_controller):
    controller, calls = sdk_controller

    controller.set_torque(2, True)
    controller.set_position(2, 1000, 1, 1021)

    assert ("protocol", 1.0) in calls
    assert ("baud", 115200) in calls
    assert ("write1", 2, module.TORQUE_ENABLE, 1) in calls
    assert ("write2", 2, module.GOAL_POSITION, 1000) in calls
    assert not any(call[0].startswith("write") and call[1] == 33 for call in calls)


def test_position_is_clamped_before_sdk_write(sdk_controller):
    controller, calls = sdk_controller

    controller.set_position(2, 5000, 1, 1021)

    assert ("write2", 2, module.GOAL_POSITION, 1021) in calls


def test_rx64_speed_uses_protocol_1_moving_speed_register(sdk_controller):
    controller, calls = sdk_controller

    controller.set_rx64_speed(50)

    assert ("write2", 33, module.MOVING_SPEED, 50) in calls
    assert not any(
        call[0] == "write2" and call[1] == 2 and call[2] == module.MOVING_SPEED
        for call in calls
    )


def test_follower_rx64_speed_uses_follower_id_and_never_rx28(sdk_controller):
    controller, calls = sdk_controller
    controller.profile = module.get_profile("follower")

    controller.set_rx64_speed(50)

    assert ("write2", 50, module.MOVING_SPEED, 50) in calls
    assert not any(
        call[0] == "write2" and call[1] == 1 and call[2] == module.MOVING_SPEED
        for call in calls
    )


@pytest.mark.parametrize(
    ("robot", "expected_range"),
    [("leader", (260, 670)), ("follower", (20, 270))],
)
def test_rx64_raw_range_matches_robot_profile(robot, expected_range):
    profile = module.get_profile(robot)

    assert (profile["rx64_min"], profile["rx64_max"]) == expected_range


def test_rx64_speed_rejects_out_of_range_value_without_write(sdk_controller):
    controller, calls = sdk_controller

    with pytest.raises(ValueError, match=r"between 0 and 1023"):
        controller.set_rx64_speed(1024)

    assert not any(
        call[0] == "write2" and call[2] == module.MOVING_SPEED for call in calls
    )


def test_transport_failure_is_reported(sdk_controller):
    controller, _ = sdk_controller
    controller.packet_handler.tx_result = 7

    with pytest.raises(module.DynamixelCommunicationError, match=r"RX28 torque: tx=7"):
        controller.set_torque(2, True)


def test_packet_error_is_reported(sdk_controller):
    controller, _ = sdk_controller
    controller.packet_handler.packet_error = 32

    with pytest.raises(
        module.DynamixelCommunicationError,
        match=r"RX28 position=450: packet=32",
    ):
        controller.set_position(2, 450, 1, 1021)


def test_rx64_speed_communication_error_is_reported(sdk_controller):
    controller, _ = sdk_controller
    controller.packet_handler.tx_result = 7

    with pytest.raises(
        module.DynamixelCommunicationError,
        match=r"RX64 moving speed=50: tx=7",
    ):
        controller.set_rx64_speed(50)


def test_rx64_speed_packet_error_is_reported(sdk_controller):
    controller, _ = sdk_controller
    controller.packet_handler.packet_error = 32

    with pytest.raises(
        module.DynamixelCommunicationError,
        match=r"RX64 moving speed=50: packet=32",
    ):
        controller.set_rx64_speed(50)
