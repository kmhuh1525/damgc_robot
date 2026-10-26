"""Clock-independent, session-scoped straight transport control (no ROS/actuators)."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    speed: float = 0.03
    acceleration: float = 0.03
    timeout: float = 0.25
    arm_timeout: float = 8.0
    max_dt: float = 0.10

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError('limits must be finite and positive')
        if self.speed > .05 or self.acceleration > .05:
            raise ValueError('straight demo speed/acceleration must be <= 0.05')


class StraightSync:
    """Leader ramps one profile; follower mirrors its signed linear command."""
    def __init__(self, role, limits=Limits()):
        if role not in ('leader', 'follower'):
            raise ValueError('invalid role')
        self.role, self.limits = role, limits
        self.state, self.reason = 'DISABLED', 'explicit enable required'
        self.session = ''
        self.armed = False
        self.value = 0.0
        self.target = 0.0
        self.input_time = None
        self.peer_time = None
        self.last_time = None
        self.seq = 0
        self.received_seq = -1
        self.status_seq = 0
        self.peer_status_seq = -1
        self.last_ack_seq = -1
        self.last_ack_sent = None
        self.sent = {}
        self.stop_until = -1.0

    @property
    def enabled(self):
        return self.state not in ('DISABLED', 'FAULT')

    def enable(self, now, session=''):
        if self.enabled:
            raise ValueError('already enabled')
        if self.role == 'leader' and not session:
            raise ValueError('leader requires a new session')
        self.state, self.reason, self.session = 'ARMING', 'arming with zero output', session
        self.armed = False
        self.value = self.target = 0.0
        self.input_time = self.peer_time = self.last_time = None
        self.seq, self.received_seq = 0, -1
        self.peer_status_seq, self.last_ack_seq = -1, -1
        self.last_ack_sent = None
        self.sent = {}
        self.enabled_at = now
        self.stop_until = -1.0
        self.active_at = None

    def armed_ok(self):
        if self.state == 'ARMING':
            self.armed = True
            self.state, self.reason = 'WAIT_PEER', 'armed; output remains zero'

    def stop(self, now, reason, fault=True):
        self.state = 'FAULT' if fault else 'DISABLED'
        self.reason, self.armed = reason, False
        self.value = self.target = 0.0
        self.input_time = None
        self.stop_until = now + .5

    def input(self, now, linear, other_axes=(0., 0., 0., 0., 0.)):
        if not self.enabled or self.role != 'leader':
            return
        values = (linear, *other_axes)
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-6 for v in other_axes):
            self.stop(now, 'nonfinite or turning/sideways command rejected')
        else:
            self.target = max(-self.limits.speed, min(self.limits.speed, float(linear)))
            self.input_time = now

    def packet(self, kind, **body):
        return dict(version=1, role=self.role, session=self.session, kind=kind, **body)

    def receive(self, packet, now):
        if not self.enabled:
            return
        try:
            if packet.get('version') != 1 or packet.get('role') == self.role:
                return
            if packet.get('role') not in ('leader', 'follower'):
                raise ValueError('invalid peer role')
            kind, session = packet.get('kind'), packet.get('session')
            if not isinstance(session, str) or not session or len(session) > 128:
                return
            if self.role == 'follower' and kind == 'PREPARE':
                if self.session and session != self.session:
                    raise ValueError('new session requires explicit follower re-enable')
                if self.state == 'WAIT_PEER' and self.armed:
                    self.session = session
                    self.enabled_at = now
                return
            if session != self.session:
                return
            if kind == 'STOP':
                self.stop(now, 'peer stopped: ' + str(packet.get('reason', 'STOP'))[:120])
                return
            if self.role == 'leader' and kind == 'STATUS':
                hseq = packet['heartbeat_seq']
                if type(hseq) is not int or hseq <= self.peer_status_seq:
                    return
                self.peer_status_seq = hseq
                if packet.get('state') not in ('WAIT_PEER', 'ACTIVE') or packet.get('armed') is not True:
                    raise ValueError('follower no longer ready')
                self.peer_time = now
                ack = packet.get('ack')
                if type(ack) is int and ack > self.last_ack_seq and ack in self.sent:
                    self.last_ack_seq = ack
                    self.last_ack_sent = self.sent[ack]
                    self.sent = {s: t for s, t in self.sent.items() if s > ack}
                if self.armed and self.state == 'WAIT_PEER':
                    self.state, self.reason, self.active_at = 'ACTIVE', 'both peers ready', now
            elif self.role == 'follower' and kind == 'COMMAND' and self.armed:
                seq, v = packet['seq'], packet['linear']
                if type(seq) is not int or seq <= self.received_seq:
                    return
                if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not -self.limits.speed - 1e-9 <= v <= self.limits.speed + 1e-9:
                    raise ValueError('invalid leader straight command')
                self.received_seq, self.peer_time = seq, now
                self.value = -float(v)
                self.state, self.reason = 'ACTIVE', 'mirroring leader profile, no deliberate delay'
        except (KeyError, TypeError, ValueError) as exc:
            self.stop(now, 'bad peer packet: ' + str(exc))

    def step(self, now):
        if self.enabled:
            if self.state != 'ACTIVE' and now - self.enabled_at > self.limits.arm_timeout and (self.role == 'leader' or self.state == 'ARMING' or self.session):
                self.stop(now, 'peer/arming timeout')
            elif self.state == 'ACTIVE':
                if self.peer_time is None or now - self.peer_time > self.limits.timeout:
                    self.stop(now, 'peer command/heartbeat timeout')
                elif self.role == 'leader':
                    if self.input_time is not None and now - self.input_time > self.limits.timeout:
                        self.stop(now, 'manual input timeout')
                    elif now - (self.last_ack_sent if self.last_ack_sent is not None else self.active_at) > self.limits.timeout:
                        self.stop(now, 'follower command acknowledgement timeout')
                    else:
                        dt = min(self.limits.max_dt, max(0., now - (self.last_time if self.last_time is not None else now)))
                        delta = self.limits.acceleration * dt
                        target = 0.0 if self.value * self.target < 0 else self.target
                        self.value += max(-delta, min(delta, target - self.value))
        self.last_time = now
        output = self.value if self.state == 'ACTIVE' else 0.0
        packets = []
        if now <= self.stop_until:
            packets.append(self.packet('STOP', reason=self.reason))
        elif self.role == 'leader' and self.enabled:
            if self.state != 'ACTIVE':
                packets.append(self.packet('PREPARE'))
            else:
                self.seq += 1
                self.sent[self.seq] = now
                packets.append(self.packet('COMMAND', seq=self.seq, linear=output))
        if self.role == 'follower':
            self.status_seq += 1
            packets.append(self.packet('STATUS', heartbeat_seq=self.status_seq, state=self.state, armed=self.armed, ack=self.received_seq))
        return output, packets
