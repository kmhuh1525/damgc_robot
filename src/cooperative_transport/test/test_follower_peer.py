"""Follower wire protocol through real ROS selector/guard, without any bridge.

Run in an isolated ROS domain. No serial/I2C node or physical actuator is used.
"""
import json
import math
import time
import tempfile
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

from cooperative_transport.hinged_formation import leader_path_to_object_path
from cooperative_transport.motion import Pose2D
from cooperative_transport.transport_peer import TransportPeer, digest
from follower_command_selector.command_selector_node import CommandSelectorNode
from follower_control.velocity_guard_node import VelocityGuardNode


class Rig:
    def __init__(self):
        self.params = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml')
        self.params.write('/follower/velocity_guard:\n  ros__parameters:\n'
                          '    command_topic: /follower/selected_cmd_vel\n'
                          '    allow_reverse: true\n')
        self.params.flush()
        rclpy.init(args=['--ros-args', '-r', '__ns:=/follower',
                        '-r', 'cmd_vel:=/follower/mission/cmd_vel',
                        '-p', 'role:=follower', '--params-file', self.params.name])
        self.selector = CommandSelectorNode()
        self.guard = VelocityGuardNode()
        self.peer = TransportPeer()
        self.driver = Node('test_leader')
        self.executor = SingleThreadedExecutor()
        for node in (self.selector, self.guard, self.peer, self.driver):
            self.executor.add_node(node)
        self.wire = self.driver.create_publisher(String, '/cooperation/transport/leader', 10)
        self.odom = self.driver.create_publisher(Odometry, '/follower/odom/raw', 10)
        self.safe, self.raw = [], []
        self.driver.create_subscription(Twist, '/follower/safe_cmd_vel',
                                        lambda m: self.safe.append((m.linear.x, m.angular.z)), 10)
        self.driver.create_subscription(Twist, '/follower/mission/cmd_vel',
                                        lambda m: self.raw.append((m.linear.x, m.angular.z)), 10)
        self.publish_odom = True
        self.heartbeat = False
        self.leader_state = 'READY'
        self.last_publish = 0.
        self.spin(.3)

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            publish = time.monotonic()-self.last_publish >= .05
            if publish:
                self.last_publish = time.monotonic()
            if self.publish_odom and publish:
                msg = Odometry()
                msg.header.frame_id = 'follower_local_odom'
                msg.pose.pose.position.x = 2.
                msg.pose.pose.position.y = 3.
                msg.pose.pose.orientation.z = 1.
                msg.pose.pose.orientation.w = 0.
                self.odom.publish(msg)
            if self.heartbeat and publish:
                self.send('HB', state=self.leader_state, progress=0., speed_ratio=1.)
            self.executor.spin_once(timeout_sec=.01)

    def wait(self, condition, timeout=2.):
        end = time.monotonic()+timeout
        while not condition() and time.monotonic() < end:
            self.spin(.03)
        assert condition(), self.peer.state

    def send(self, kind, **values):
        self.wire.publish(String(data=json.dumps(dict(
            v=1, kind=kind, session=self.peer.session, hash=self.peer.path_hash, **values))))

    def packet(self):
        leader = tuple(Pose2D(i*.025, 0., 0.) for i in range(21))
        _, formation = leader_path_to_object_path(leader, self.peer.geometry)
        g = self.peer.geometry
        body = dict(frame='leader_local_odom', geometry=[g.axle_to_hinge,
                    g.hinge_to_contact, g.object_center_to_contact, g.hinge_limit],
                    leader=[[p.x,p.y,p.yaw] for p in formation.leader],
                    follower=[[p.x,p.y,p.yaw] for p in formation.follower],
                    expected_follower=[.62, 0., math.pi], speed=.05)
        return dict(v=1, kind='PREPARE', session='test-session', hash=digest(body),
                    body=body, nonce='test-nonce', t1=time.time())

    def prepare(self):
        self.wire.publish(String(data=json.dumps(self.packet())))
        self.heartbeat = True
        self.wait(lambda: self.peer.state == 'READY')

    def start_simulated(self):
        self.peer.motion_enabled = True  # No STM32 bridge exists in this rig.
        self.prepare()
        self.send('ARM')
        self.wait(lambda: self.peer.state == 'ARMED')
        self.leader_state = 'SCHEDULED'
        self.send('COMMIT', start=time.time()+.8, offset=0.)
        self.wait(lambda: self.peer.state == 'RUNNING')
        self.wait(lambda: any(x < -.001 for x, _ in self.safe))

    def close(self):
        self.peer.stop('test teardown')
        self.executor.shutdown()
        for node in (self.peer, self.guard, self.selector, self.driver):
            node.destroy_node()
        rclpy.shutdown()
        self.params.close()


@pytest.fixture
def rig():
    instance = Rig()
    try:
        yield instance
    finally:
        instance.close()


def test_prepare_maps_path_to_local_odom_without_motion(rig):
    rig.prepare()
    rig.spin(.2)
    assert rig.peer.odom_frame == 'follower_local_odom'
    assert rig.peer.path[0].x == pytest.approx(2.)
    assert rig.peer.path[-1].x == pytest.approx(2.5)
    assert rig.selector.get_parameter('source_mode').value == 'STOP'
    assert rig.safe and all(v == (0., 0.) for v in rig.safe)


def test_default_motion_gate_rejects_arm(rig):
    rig.prepare()
    rig.send('ARM')
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.spin(.15)
    assert all(v == (0., 0.) for v in rig.safe)


def test_simulated_scheduled_reverse_and_stop(rig):
    rig.peer.motion_enabled = True  # Test only: there is no STM32 bridge.
    rig.prepare()
    rig.send('ARM')
    rig.wait(lambda: rig.peer.state == 'ARMED')
    assert all(v == (0., 0.) for v in rig.safe)
    rig.leader_state = 'SCHEDULED'
    rig.send('COMMIT', start=time.time()+.8, offset=0.)
    rig.wait(lambda: rig.peer.state == 'RUNNING')
    rig.wait(lambda: any(x < -.001 for x, _ in rig.safe))
    rig.send('STOP', detail='operator abort')
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.spin(.3)
    assert rig.selector.get_parameter('source_mode').value == 'STOP'
    assert rig.safe[-1] == (0., 0.)


def test_tampered_hash_never_ready(rig):
    packet = rig.packet()
    packet['body']['speed'] = .1
    rig.wire.publish(String(data=json.dumps(packet)))
    rig.spin(.2)
    assert rig.peer.state == 'IDLE'
    assert all(v == (0., 0.) for v in rig.safe)


def test_geometry_mismatch_reports_stop(rig):
    packet = rig.packet()
    packet['body']['geometry'][0] = .2
    packet['hash'] = digest(packet['body'])
    rig.wire.publish(String(data=json.dumps(packet)))
    rig.wait(lambda: rig.peer.state == 'STOPPED')


@pytest.mark.parametrize('lost', ['odometry', 'heartbeat'])
def test_loss_closes_ready_session(rig, lost):
    rig.prepare()
    if lost == 'odometry':
        rig.publish_odom = False
    else:
        rig.heartbeat = False
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.spin(.15)
    assert rig.safe[-1] == (0., 0.)


def test_conflicting_command_publisher_rejects_prepare(rig):
    conflict = rig.driver.create_publisher(Twist, '/follower/mission/cmd_vel', 10)
    rig.spin(.15)
    rig.wire.publish(String(data=json.dumps(rig.packet())))
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.driver.destroy_publisher(conflict)


def test_unresponsive_service_times_out(rig, monkeypatch):
    rig.prepare()
    monkeypatch.setattr(rig.peer.selector, 'call_async', lambda request: Future())
    rig.peer.source('STOP', lambda: pytest.fail('unanswered service succeeded'))
    rig.wait(lambda: rig.peer.state == 'STOPPED', timeout=3.)


def test_non_object_packet_does_not_crash(rig):
    rig.wire.publish(String(data='[]'))
    rig.spin(.15)
    assert rig.peer.state == 'IDLE'


@pytest.mark.parametrize('lost', ['odometry', 'heartbeat'])
def test_loss_stops_running_output(rig, lost):
    rig.start_simulated()
    if lost == 'odometry':
        rig.publish_odom = False
    else:
        rig.heartbeat = False
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.spin(.3)
    assert rig.selector.get_parameter('source_mode').value == 'STOP'
    assert rig.safe[-1] == (0., 0.)


def test_late_commit_rejected_without_motion(rig):
    rig.peer.motion_enabled = True
    rig.prepare()
    rig.send('ARM')
    rig.wait(lambda: rig.peer.state == 'ARMED')
    rig.send('COMMIT', start=time.time()+.1, offset=0.)
    rig.wait(lambda: rig.peer.state == 'STOPPED')
    rig.spin(.15)
    assert all(v == (0., 0.) for v in rig.safe)


def test_valid_hash_with_wrong_follower_path_rejected(rig):
    packet = rig.packet()
    packet['body']['follower'][5][1] += .05
    packet['hash'] = digest(packet['body'])
    rig.wire.publish(String(data=json.dumps(packet)))
    rig.wait(lambda: rig.peer.state == 'STOPPED')


def test_real_leader_transition_packet_reaches_ready_without_motion(rig):
    fixture = json.loads((Path(__file__).parent/'fixtures/leader_d812a04_paths.json').read_text())
    packet = rig.packet()
    packet['body'] = fixture['cases'][1]['body']
    packet['hash'] = digest(packet['body'])
    rig.wire.publish(String(data=json.dumps(packet)))
    rig.heartbeat = True
    rig.wait(lambda: rig.peer.state == 'READY')
    rig.spin(.1)
    assert rig.selector.get_parameter('source_mode').value == 'STOP'
    assert all(v == (0.,0.) for v in rig.safe)
