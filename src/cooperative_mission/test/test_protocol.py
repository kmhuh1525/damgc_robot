import json

import pytest

from cooperative_mission.protocol import (
    CommandMessage,
    FollowerCommand,
    FollowerState,
    StatusMessage,
    decode_command,
    decode_status,
    encode_command,
    encode_status,
)


def test_command_round_trip() -> None:
    message = CommandMessage("Labc", 7, FollowerCommand.LIFT)
    assert decode_command(encode_command(message)) == message


def test_status_round_trip() -> None:
    message = StatusMessage("Labc", 3, FollowerState.LIFT_READY, True, "armed")
    assert decode_status(encode_status(message)) == message


@pytest.mark.parametrize(
    "payload",
    [
        "not json",
        json.dumps([1, 2]),
        json.dumps({"v": 2, "session": "L", "seq": 1, "command": "LIFT"}),
        json.dumps({"v": 1, "session": "", "seq": 1, "command": "LIFT"}),
        json.dumps({"v": 1, "session": "L", "seq": 0, "command": "LIFT"}),
        json.dumps({"v": 1, "session": "L", "seq": True, "command": "LIFT"}),
        json.dumps({"v": 1, "session": "L", "seq": 1.5, "command": "LIFT"}),
        json.dumps({"v": 1, "session": "L", "seq": 1, "command": "JUMP"}),
        "x" * 5000,
    ],
)
def test_malformed_commands_are_rejected(payload: str) -> None:
    assert decode_command(payload) is None


@pytest.mark.parametrize(
    "payload",
    [
        json.dumps({"v": 1, "session": "L", "ack_seq": -1, "state": "IDLE", "ready": True}),
        json.dumps({"v": 1, "session": "L", "ack_seq": 1, "state": "NOPE", "ready": True}),
        json.dumps({"v": 1, "session": "L", "ack_seq": 1, "state": "IDLE", "ready": "yes"}),
        json.dumps({"v": 1, "session": 5, "ack_seq": 1, "state": "IDLE", "ready": True}),
    ],
)
def test_malformed_status_is_rejected(payload: str) -> None:
    assert decode_status(payload) is None
