"""Side-effect requests emitted by the ROS-independent mission logic.

The logic never talks to ROS directly. It emits :class:`Action` objects which
the node executes (service call, parameter set or topic publish) and then
reports back with :meth:`on_action_result`. Actions created with
``wait=True`` must be confirmed before the logic starts motion that depends
on them; a missing confirmation turns into a FAULT after ``action_timeout``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class ActionKind(str, Enum):
    APPROACH_ENABLE = "APPROACH_ENABLE"  # value: bool  -> <ns>/approach/enable
    GUARD_ENABLE = "GUARD_ENABLE"  # value: bool  -> <ns>/velocity_guard/enable
    SELECTOR_MODE = "SELECTOR_MODE"  # value: str   -> command_selector source_mode
    GRIPPER = "GRIPPER"  # value: tuple -> <ns>/dynamixel/command


@dataclass(frozen=True)
class Action:
    action_id: int
    kind: ActionKind
    value: object
    wait: bool

    @property
    def is_safety_release(self) -> bool:
        """Disabling motion is best-effort: its failure must not re-fault."""
        if self.kind in (ActionKind.APPROACH_ENABLE, ActionKind.GUARD_ENABLE):
            return self.value is False
        if self.kind == ActionKind.SELECTOR_MODE:
            return self.value == "STOP"
        return False


@dataclass
class ActionTracker:
    """Queue actions and remember which confirmations are still missing."""

    timeout: float
    _next_id: int = 1
    _queue: List[Action] = field(default_factory=list)
    _awaiting: Dict[int, Tuple[Action, float]] = field(default_factory=dict)

    def emit(self, kind: ActionKind, value: object, now: float, wait: bool = True) -> Action:
        action = Action(self._next_id, kind, value, wait)
        self._next_id += 1
        self._queue.append(action)
        if wait:
            self._awaiting[action.action_id] = (action, now)
        return action

    def drain(self) -> List[Action]:
        actions, self._queue = self._queue, []
        return actions

    def resolve(self, action_id: int) -> Optional[Action]:
        entry = self._awaiting.pop(action_id, None)
        return None if entry is None else entry[0]

    def clear_waits(self) -> None:
        self._awaiting.clear()

    @property
    def busy(self) -> bool:
        return bool(self._awaiting)

    def expired(self, now: float) -> Optional[Action]:
        for action, issued in self._awaiting.values():
            if now - issued > self.timeout:
                return action
        return None
