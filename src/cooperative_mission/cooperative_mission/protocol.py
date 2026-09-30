"""Leader<->Follower mission messages carried as JSON in ``std_msgs/String``.

Only standard ROS message types cross the Orin-Orin DDS link so that either
robot can be rebuilt independently (the same policy ``leader_cooperation``
uses). The payload is versioned and validated on receipt; malformed input is
ignored rather than interpreted.

Leader -> Follower  ``/mission/follower_command``
    {"v": 1, "session": "<leader run id>", "seq": <int>, "command": "<CMD>"}

Follower -> Leader  ``/follower/mission/status``
    {"v": 1, "session": "<last leader session>", "ack_seq": <int>,
     "state": "<STATE>", "ready": <bool>, "detail": "<text>"}

Commands are re-sent until the Follower acknowledges ``seq``. The Follower
executes each ``(session, seq)`` exactly once and acknowledges duplicates
again, so retransmission is idempotent.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Optional

PROTOCOL_VERSION = 1


class FollowerCommand(str, Enum):
    """Commands the Leader mission may issue to the Follower mission."""

    APPROACH = "APPROACH"
    PREPARE_LIFT = "PREPARE_LIFT"
    LIFT = "LIFT"
    PREPARE_TRANSPORT = "PREPARE_TRANSPORT"
    END_TRANSPORT = "END_TRANSPORT"
    RELEASE = "RELEASE"
    ABORT = "ABORT"
    RESET = "RESET"


class FollowerState(str, Enum):
    """Externally visible Follower mission states."""

    IDLE = "IDLE"
    REPOSITION = "REPOSITION"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    GRASPED = "GRASPED"
    LIFT_READY = "LIFT_READY"
    LIFTING = "LIFTING"
    LIFTED = "LIFTED"
    TRANSPORT_READY = "TRANSPORT_READY"
    HOLD = "HOLD"
    RELEASING = "RELEASING"
    FAULT = "FAULT"


@dataclass(frozen=True)
class CommandMessage:
    session: str
    seq: int
    command: FollowerCommand


@dataclass(frozen=True)
class StatusMessage:
    session: str
    ack_seq: int
    state: FollowerState
    ready: bool
    detail: str = ""


def encode_command(message: CommandMessage) -> str:
    return json.dumps(
        {
            "v": PROTOCOL_VERSION,
            "session": message.session,
            "seq": int(message.seq),
            "command": message.command.value,
        },
        separators=(",", ":"),
    )


def decode_command(text: str) -> Optional[CommandMessage]:
    """Return a validated command or ``None`` for anything malformed."""
    data = _load(text)
    if data is None:
        return None
    session = data.get("session")
    seq = data.get("seq")
    command = data.get("command")
    if not isinstance(session, str) or not session:
        return None
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
        return None
    try:
        parsed = FollowerCommand(command)
    except ValueError:
        return None
    return CommandMessage(session=session, seq=seq, command=parsed)


def encode_status(message: StatusMessage) -> str:
    return json.dumps(
        {
            "v": PROTOCOL_VERSION,
            "session": message.session,
            "ack_seq": int(message.ack_seq),
            "state": message.state.value,
            "ready": bool(message.ready),
            "detail": message.detail,
        },
        separators=(",", ":"),
    )


def decode_status(text: str) -> Optional[StatusMessage]:
    """Return a validated Follower status or ``None`` for malformed input."""
    data = _load(text)
    if data is None:
        return None
    session = data.get("session", "")
    ack_seq = data.get("ack_seq")
    ready = data.get("ready")
    detail = data.get("detail", "")
    if not isinstance(session, str):
        return None
    if isinstance(ack_seq, bool) or not isinstance(ack_seq, int) or ack_seq < 0:
        return None
    if not isinstance(ready, bool):
        return None
    if not isinstance(detail, str):
        detail = str(detail)
    try:
        state = FollowerState(data.get("state"))
    except ValueError:
        return None
    return StatusMessage(
        session=session, ack_seq=ack_seq, state=state, ready=ready, detail=detail
    )


def _load(text: str) -> Optional[dict]:
    if not isinstance(text, str) or len(text) > 4096:
        return None
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("v") != PROTOCOL_VERSION:
        return None
    return data
