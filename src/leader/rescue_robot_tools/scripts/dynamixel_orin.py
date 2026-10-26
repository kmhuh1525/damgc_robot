#!/usr/bin/env python3
"""Dynamixel Protocol 1.0 controller for RX-64 and RX-28 through U2D2.

The ROBOTIS SDK owns serial direction control, packet construction, and
status-packet validation while this wrapper preserves the existing interface.
"""

import argparse

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


BAUDRATE = 115200
PROTOCOL_VERSION = 1.0
TORQUE_ENABLE = 24
GOAL_POSITION = 30
MOVING_SPEED = 32
MAX_JOINT_MODE_SPEED = 1023

PROFILES = {
    "leader": {
        "rx64_id": 33, "rx64_min": 260, "rx64_max": 670,
        "rx28_id": 2, "rx28_min": 1, "rx28_max": 1021,
    },
    "follower": {
        # Physical endpoints confirmed on the follower hardware.
        "rx64_id": 50, "rx64_min": 45, "rx64_max": 286,
        "rx28_id": 1, "rx28_min": 278, "rx28_max": 1000,
        "rx28_open": 1000, "rx28_close": 331,
    },
}


class DynamixelCommunicationError(RuntimeError):
    """A Dynamixel write failed at transport or packet-error level."""


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


def get_profile(robot: str) -> dict:
    try:
        return PROFILES[robot].copy()
    except KeyError as exc:
        raise ValueError(f"unknown robot profile: {robot}") from exc


class DynamixelOrin:
    def __init__(self, port: str, profile: dict, baudrate: int = BAUDRATE):
        self.profile = profile
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self._closed = False

        if not self.port_handler.openPort():
            raise DynamixelCommunicationError(f"open port failed: {port}")
        if not self.port_handler.setBaudRate(baudrate):
            self.close()
            raise DynamixelCommunicationError(f"set baudrate failed: {baudrate}")

    def close(self):
        if not self._closed:
            self.port_handler.closePort()
            self._closed = True

    def _label(self, device_id: int) -> str:
        if device_id == self.profile["rx64_id"]:
            return "RX64"
        if device_id == self.profile["rx28_id"]:
            return "RX28"
        return f"ID{device_id}"

    def _check_result(self, operation: str, comm_result: int, packet_error: int):
        if comm_result != COMM_SUCCESS:
            detail = self.packet_handler.getTxRxResult(comm_result)
            raise DynamixelCommunicationError(f"{operation}: {detail}")
        if packet_error != 0:
            detail = self.packet_handler.getRxPacketError(packet_error)
            raise DynamixelCommunicationError(f"{operation}: {detail}")

    def set_torque(self, device_id: int, enabled: bool):
        operation = f"{self._label(device_id)} torque"
        result, packet_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, device_id, TORQUE_ENABLE, 1 if enabled else 0
        )
        self._check_result(operation, result, packet_error)

    def set_position(self, device_id: int, position: int, minimum: int, maximum: int):
        position = clamp(position, minimum, maximum)
        operation = f"{self._label(device_id)} position={position}"
        result, packet_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, device_id, GOAL_POSITION, position
        )
        self._check_result(operation, result, packet_error)

    def set_rx64_speed(self, speed: int):
        self.set_moving_speed(self.profile["rx64_id"], speed)

    def set_rx28_speed(self, speed: int):
        self.set_moving_speed(self.profile["rx28_id"], speed)

    def set_moving_speed(self, device_id: int, speed: int):
        if not 0 <= speed <= MAX_JOINT_MODE_SPEED:
            raise ValueError(
                f"moving speed must be between 0 and {MAX_JOINT_MODE_SPEED}"
            )
        operation = f"{self._label(device_id)} moving speed={speed}"
        result, packet_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, device_id, MOVING_SPEED, speed
        )
        self._check_result(operation, result, packet_error)


def main():
    parser = argparse.ArgumentParser(description="Direct U2D2 RX-64/RX-28 controller")
    parser.add_argument("--port", required=True, help="Orin USB/RS-485 port, e.g. /dev/ttyUSB0")
    parser.add_argument("--robot", choices=sorted(PROFILES), default="leader")
    parser.add_argument("--rx64-raw", type=int, help="RX-64 raw goal position")
    parser.add_argument("--rx28-raw", type=int, help="RX-28 raw goal position")
    parser.add_argument("--rx64-speed", type=int, default=10, help="RX-64 Moving Speed raw value")
    parser.add_argument("--rx28-speed", type=int, default=40, help="RX-28 Moving Speed raw value")
    parser.add_argument("--torque", action="store_true", help="enable torque before moving")
    parser.add_argument("--no-torque", action="store_true", help="disable torque")
    args = parser.parse_args()
    profile = get_profile(args.robot)

    if args.torque and args.no_torque:
        parser.error("--torque and --no-torque cannot be used together")
    if args.rx64_raw is None and args.rx28_raw is None and not (args.torque or args.no_torque):
        parser.error("provide a raw goal position or torque option")

    controller = DynamixelOrin(args.port, profile)
    try:
        controller.set_rx64_speed(args.rx64_speed)
        controller.set_rx28_speed(args.rx28_speed)
        if args.no_torque:
            controller.set_torque(profile["rx64_id"], False)
            controller.set_torque(profile["rx28_id"], False)
        else:
            if args.torque:
                controller.set_torque(profile["rx64_id"], True)
                controller.set_torque(profile["rx28_id"], True)
            if args.rx64_raw is not None:
                controller.set_position(
                    profile["rx64_id"], args.rx64_raw,
                    profile["rx64_min"], profile["rx64_max"]
                )
            if args.rx28_raw is not None:
                controller.set_position(
                    profile["rx28_id"], args.rx28_raw,
                    profile["rx28_min"], profile["rx28_max"]
                )
    finally:
        controller.close()


if __name__ == "__main__":
    main()
