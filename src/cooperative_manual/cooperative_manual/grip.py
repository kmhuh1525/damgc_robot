"""Idempotent paired gripper commands. No bus ownership or lift commands."""
import math
import uuid


class PairedGrip:
    def __init__(self, role, enabled=False, open_raw=1000, close_raw=-1, settle=9.):
        lower,upper=(278,1000) if role=='follower' else (1,1021)
        if enabled and (not lower <= open_raw <= upper or not lower <= close_raw <= upper):
            raise ValueError('configure calibrated open_raw and close_raw before enabling gripper')
        if not math.isfinite(settle) or not 0 < settle <= 60:
            raise ValueError('gripper settle must be in (0,60] seconds')
        self.role, self.enabled = role, enabled
        self.open_raw, self.close_raw, self.settle = open_raw, close_raw, settle
        self.phase, self.reason = 'IDLE', 'G=grasp, R=release; stopped base required'
        self.action_id, self.action, self.session = '', '', ''
        self.motor_ack = self.peer_ack = self.peer_done = False
        self.commands, self.packets = [], []
        self.last_send = -1.
        self.started = self.issued = 0.

    @property
    def busy(self):
        return self.phase not in ('IDLE', 'DONE', 'ERROR')

    @property
    def target(self):
        return self.close_raw if self.action == 'GRASP' else self.open_raw

    def packet(self, kind, **values):
        return dict(version=1, role=self.role, session=self.session, kind=kind, action_id=self.action_id, action=self.action, **values)

    def fail(self, reason):
        if self.busy:
            self.phase, self.reason = 'ERROR', str(reason)
            self.commands.clear()

    def request(self, action, session, now):
        if self.role != 'leader' or not self.enabled or action not in ('GRASP', 'RELEASE') or self.busy:
            raise ValueError('gripper disabled, invalid action or action already running')
        self.action, self.session, self.action_id = action, session, uuid.uuid4().hex
        self.phase, self.reason = 'STOPPING', 'stopping both bases before ' + action
        self.started, self.last_send = now, -1.
        self.motor_ack = self.peer_ack = self.peer_done = False
        self.commands.clear();self.packets.clear()

    def _execute(self, now):
        self.commands.append(self.target)
        self.issued, self.motor_ack = now, False
        self.phase, self.reason = 'WAIT_ACK', 'command sent; waiting for motor ACK'

    def receive(self, packet, session, now, stationary, motor_ready):
        if packet.get('version') != 1 or packet.get('session') != session or packet.get('role') != ('leader' if self.role=='follower' else 'follower'):
            return
        aid, action, kind = packet.get('action_id'), packet.get('action'), packet.get('kind')
        if not isinstance(aid,str) or not aid or len(aid)>128 or action not in ('GRASP','RELEASE'):
            return
        if self.role == 'follower' and kind == 'GRIP_PREPARE':
            if aid == self.action_id:
                if self.phase == 'READY':self.packets.append(self.packet('GRIP_READY'))
                elif self.phase == 'DONE':self.packets.append(self.packet('GRIP_DONE'))
                return
            if self.busy or not self.enabled or not motor_ready:
                self.packets.append(dict(version=1, role=self.role, session=session, kind='GRIP_REJECT', action_id=aid, action=action, reason='follower not stationary/configured or motor unavailable'))
                return
            if not stationary:
                return  # Leader retries PREPARE while the follower finishes stopping.
            self.action_id, self.action, self.session = aid, action, session
            self.started, self.last_send = now, -1.
            self.motor_ack = self.peer_ack = self.peer_done = False
            self.phase, self.reason = 'READY', 'base stopped; motor present; awaiting commit'
            self.packets.append(self.packet('GRIP_READY'))
            return
        if aid != self.action_id or session != self.session or action != self.action:
            return
        if self.role == 'leader':
            if kind == 'GRIP_REJECT':self.fail(packet.get('reason','peer rejected action'))
            elif kind == 'GRIP_READY' and self.phase == 'PREPARING':
                if not stationary or not motor_ready:
                    self.fail('leader no longer stationary or motor unavailable')
                else:
                    # One commit for both motors; retries never execute again.
                    self._execute(now)
                    self.packets.append(self.packet('GRIP_COMMIT'))
            elif kind in ('GRIP_ACK','GRIP_DONE') and self.phase in ('WAIT_ACK','SETTLING'):
                self.peer_ack = True
                self.peer_done = kind == 'GRIP_DONE' or self.peer_done
        elif kind == 'GRIP_COMMIT':
            if self.phase == 'READY':
                if not stationary or not motor_ready:self.fail('follower no longer stationary or motor unavailable')
                else:self._execute(now)
            elif self.phase == 'DONE':self.packets.append(self.packet('GRIP_DONE'))
            elif self.motor_ack:self.packets.append(self.packet('GRIP_ACK'))

    def motor_status(self, text):
        if not self.busy:
            return
        if text.startswith('ERROR'):
            self.fail('motor: '+text)
        elif self.phase in ('WAIT_ACK','SETTLING') and text.startswith('OK ') and ('rx28=%d '%self.target) in text:
            self.motor_ack = True

    def tick(self, now, stationary, motor_ready):
        if self.busy and now-self.started>90:
            self.fail('gripper transaction timeout')
        if self.phase=='STOPPING' and now-self.started>3:
            self.fail('bases did not stop before gripper action')
        if self.phase=='READY' and now-self.started>3:
            self.fail('gripper commit timeout')
        if self.phase=='STOPPING' and stationary:
            if not motor_ready:self.fail('leader motor unavailable')
            else:self.phase,self.reason='PREPARING','both bases stopped; waiting for follower readiness'
        if self.phase=='PREPARING' and now-self.started>8:
            self.fail('follower gripper readiness timeout')
        if self.phase in ('WAIT_ACK','SETTLING'):
            if not self.motor_ack and now-self.issued>3:self.fail('motor ACK timeout')
            elif self.role=='leader' and not self.peer_ack and now-self.issued>3:self.fail('peer motor ACK timeout')
            elif self.motor_ack and (self.role=='follower' or self.peer_ack):
                self.phase,self.reason='SETTLING','both command receipt and movement wait in progress'
                if now-self.issued>=self.settle and (self.role=='follower' or self.peer_done):
                    self.phase,self.reason='DONE',self.action+' command acknowledged and settle wait completed'
        if now-self.last_send>=.1:
            self.last_send=now
            if self.role=='leader' and self.phase=='PREPARING':self.packets.append(self.packet('GRIP_PREPARE'))
            elif self.role=='leader' and self.phase in ('WAIT_ACK','SETTLING') and not self.peer_ack:self.packets.append(self.packet('GRIP_COMMIT'))
            elif self.role=='follower' and self.phase=='READY':self.packets.append(self.packet('GRIP_READY'))
            elif self.role=='follower' and self.phase in ('WAIT_ACK','SETTLING') and self.motor_ack:self.packets.append(self.packet('GRIP_ACK'))
            elif self.role=='follower' and self.phase=='DONE':self.packets.append(self.packet('GRIP_DONE'))
        commands,self.commands=self.commands,[]
        packets,self.packets=self.packets,[]
        return commands,packets
