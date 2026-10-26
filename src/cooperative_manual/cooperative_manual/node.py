"""Paired straight-drive and gripper orchestration; no direct hardware connection."""
import json
import math
import time
import signal
import threading
import uuid

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import SetParameters, GetParameters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import SetBool

from .logic import Limits, StraightSync
from .grip import PairedGrip


class ManualStraightNode(Node):
    def __init__(self, **kwargs):
        super().__init__('manual_straight', **kwargs)
        for name, value in {'role':'leader', 'motion_enabled':False, 'speed':.03, 'acceleration':.03, 'timeout':.25, 'command_topic':''}.items():
            self.declare_parameter(name, value)
        self.role = self.get_parameter('role').value
        self.logic = StraightSync(self.role, Limits(speed=self.get_parameter('speed').value, acceleration=self.get_parameter('acceleration').value, timeout=self.get_parameter('timeout').value))
        self.declare_parameter('odom_topic', '/leader/odometry/local' if self.role == 'leader' else '/follower/odom/raw')
        self.odom_time = None
        self.odom_stationary = False
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odometry, rclpy.qos.qos_profile_sensor_data)
        self.command_topic = self.get_parameter('command_topic').value or ('/leader/cooperation/cmd_vel' if self.role == 'leader' else '/follower/mission/cmd_vel')
        self.cmd = self.create_publisher(Twist, self.command_topic, 1)
        self.wire = self.create_publisher(String, '/cooperation/manual/' + self.role, 1)
        self.status = self.create_publisher(String, '/cooperation/manual/' + self.role + '/status', 10)
        self.create_subscription(String, '/cooperation/manual/' + ('follower' if self.role == 'leader' else 'leader'), self.receive, 1)
        self.create_subscription(ParameterEvent, '/parameter_events', self.ownership, 10)
        self.selector_name = '/' + self.role + '/command_selector'
        self.selector = self.create_client(SetParameters, self.selector_name + '/set_parameters')
        self.guard = self.create_client(SetBool, '/follower/velocity_guard/enable') if self.role == 'follower' else None
        self.guard_config = self.create_client(GetParameters,'/follower/velocity_guard/get_parameters') if self.guard else None
        self.future_kind = ''
        self.create_service(SetBool, 'manual_straight/enable', self.enable)
        if self.role == 'leader':
            self.create_subscription(Twist, '/cooperation/manual/input', self.manual, 1)
        self.controls = []
        self.future = None
        self.future_deadline = 0.
        self.released = True
        self.last_state = None
        for name, value in {'gripper_enabled':False, 'open_raw':1000, 'close_raw':331 if self.role=='follower' else -1, 'gripper_settle':9. if self.role=='follower' else 25.}.items():
            self.declare_parameter(name,value)
        self.grip = PairedGrip(self.role,self.get_parameter('gripper_enabled').value,self.get_parameter('open_raw').value,self.get_parameter('close_raw').value,self.get_parameter('gripper_settle').value)
        self.motor = self.create_publisher(Float64MultiArray,'/'+self.role+'/dynamixel/command',10)
        self.create_subscription(String,'/'+self.role+'/dynamixel/status',lambda m:self.grip.motor_status(m.data),10)
        self.grip_wire = self.create_publisher(String,'/cooperation/manual/gripper/'+self.role,10)
        self.grip_status = self.create_publisher(String,'/cooperation/manual/gripper/'+self.role+'/status',10)
        self.create_subscription(String,'/cooperation/manual/gripper/'+('follower' if self.role=='leader' else 'leader'),self.grip_receive,10)
        if self.role=='leader':
            self.create_subscription(String,'/cooperation/manual/gripper/request',self.grip_request,10)
        self.need_neutral = False
        self.last_grip_state = None
        self.last_health = 0.
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info('straight-only transport: motion_enabled=%s; explicit enable required' % self.get_parameter('motion_enabled').value)

    def odometry(self, msg):
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not all(math.isfinite(v) for v in values) or not 0 <= age <= .5 or stamp <= 0:
            return
        self.odom_time = time.monotonic()
        self.odom_stationary = abs(twist.linear.x) <= .01 and abs(twist.angular.z) <= .03

    def publish_zero(self):
        self.cmd.publish(Twist())

    def source_request(self, source):
        params = [Parameter('source_mode', value=source).to_parameter_msg()]
        if self.role == 'leader':
            params.append(Parameter('enable_nav2_goal_selection', value=False).to_parameter_msg())
        return self.selector.call_async(SetParameters.Request(parameters=params))

    def release(self, change_selector=True):
        self.publish_zero()
        self.controls = []
        self.future = None
        if self.guard and self.guard.service_is_ready():
            self.guard.call_async(SetBool.Request(data=False))
        if change_selector and self.selector.service_is_ready():
            self.source_request('STOP')
        self.released = True

    def ownership(self, event):
        if self.logic.state != 'ACTIVE' or event.node != self.selector_name:
            return
        for param in list(event.changed_parameters) + list(event.new_parameters):
            if param.name == 'source_mode' and param.value.string_value != 'COOPERATION':
                self.logic.stop(time.monotonic(), 'selector ownership changed')
                self.release(change_selector=False)

    def enable(self, request, response):
        now = time.monotonic()
        if not request.data:
            self.logic.stop(now, 'operator stop', fault=False)
            self.release()
            response.success, response.message = True, 'stopped; new explicit enable required on both peers'
            return response
        if not self.get_parameter('motion_enabled').value:
            response.message = 'motion disabled; launch motion_enabled:=true only for a prepared test'
            return response
        if self.logic.enabled:
            response.message = 'already enabled'
            return response
        if self.odom_time is None or now - self.odom_time > .35 or not self.odom_stationary:
            response.message = 'fresh stationary odometry required before enable'
            return response
        if not self.selector.service_is_ready() or (self.guard and (not self.guard.service_is_ready() or not self.guard_config.service_is_ready())):
            response.message = 'selector/guard service unavailable'
            return response
        if self.count_publishers(self.command_topic) != 1:
            response.message = 'conflicting command publisher; stop other transport/missions first'
            return response
        if self.count_publishers('/cooperation/manual/' + self.role) != 1:
            response.message = 'duplicate manual peer'
            return response
        if self.grip.phase=='ERROR':
            self.grip=PairedGrip(self.role,self.grip.enabled,self.grip.open_raw,self.grip.close_raw,self.grip.settle)
        self.logic.enable(now, uuid.uuid4().hex if self.role == 'leader' else '')
        self.released = False
        self.controls = (['guard_config'] if self.guard else []) + ['selector'] + (['guard'] if self.guard else [])
        self.future = None
        response.success, response.message = True, 'arming; no motion until both peers are ready and fresh manual input arrives'
        return response

    def stationary(self):
        return self.odom_time is not None and time.monotonic()-self.odom_time<=.35 and self.odom_stationary and abs(self.logic.value)<1e-6

    def motor_ready(self):
        return self.motor.get_subscription_count()==1

    def grip_request(self,msg):
        try:
            if self.logic.state!='ACTIVE':raise ValueError('enable both peers before paired grip')
            self.grip.request(msg.data,self.logic.session,time.monotonic())
            self.need_neutral=True
            self.logic.input(time.monotonic(),0.)
        except ValueError as exc:
            self.grip_status.publish(String(data='REJECTED: '+str(exc)))

    def grip_receive(self,msg):
        try:
            if self.logic.state!='ACTIVE' or len(msg.data)>4096:return
            packet=json.loads(msg.data)
            if not isinstance(packet,dict):raise ValueError('gripper packet must be object')
            self.grip.receive(packet,self.logic.session,time.monotonic(),self.stationary(),self.motor_ready())
        except (ValueError,TypeError) as exc:
            self.logic.stop(time.monotonic(),'invalid gripper packet: '+str(exc))

    def manual(self, msg):
        value=msg.linear.x
        if self.grip.busy:
            value=0.
        elif self.need_neutral:
            if abs(value)<1e-6:self.need_neutral=False
            value=0.
        self.logic.input(time.monotonic(), value, (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z))

    def receive(self, msg):
        if len(msg.data) > 4096:
            if self.logic.enabled:
                self.logic.stop(time.monotonic(), 'oversized peer packet')
            return
        try:
            packet = json.loads(msg.data)
            if not isinstance(packet, dict):
                raise ValueError('packet must be object')
            if self.role=='follower' and self.grip.busy and packet.get('kind')=='COMMAND' and abs(packet.get('linear',0.))>1e-6:
                self.logic.stop(time.monotonic(),'movement requested during gripper action')
            else:
                self.logic.receive(packet, time.monotonic())
        except (ValueError, TypeError) as exc:
            if self.logic.enabled:
                self.logic.stop(time.monotonic(), 'invalid packet: ' + str(exc))

    def tick(self):
        now = time.monotonic()
        if self.logic.enabled and now - self.last_health >= .1:
            self.last_health = now
            if self.odom_time is None or now - self.odom_time > .35:
                self.logic.stop(now, 'odometry timeout')
            if self.role == 'leader' and self.count_publishers('/cooperation/manual/input') > 1:
                self.logic.stop(now, 'conflicting manual input publishers')
            if self.count_publishers(self.command_topic) != 1 or self.count_publishers('/cooperation/manual/' + self.role) != 1:
                self.logic.stop(now, 'conflicting publisher')
        if self.logic.state == 'ARMING':
            try:
                if self.future is not None:
                    if self.future.done():
                        result = self.future.result()
                        if self.future_kind=='guard_config':
                            okay=result is not None and len(result.values)==1 and result.values[0].type==1 and result.values[0].bool_value
                        else:
                            okay = result is not None and (bool(result.results) and all(x.successful for x in result.results) if hasattr(result, 'results') else result.success)
                        if not okay:
                            raise ValueError('arming rejected: require reverse-capable follower guard and COOPERATION selector')
                        self.future = None
                    elif now > self.future_deadline:
                        raise ValueError('selector/guard service timeout')
                if self.future is None:
                    if self.controls:
                        action = self.controls.pop(0)
                        self.future_kind=action
                        if action=='guard_config':self.future=self.guard_config.call_async(GetParameters.Request(names=['allow_reverse']))
                        elif action=='selector':self.future=self.source_request('COOPERATION')
                        else:self.future=self.guard.call_async(SetBool.Request(data=True))
                        self.future_deadline = now + 2.
                    else:
                        self.logic.armed_ok()
            except Exception as exc:
                self.logic.stop(now, str(exc))
        if not self.logic.enabled and self.grip.busy:
            self.grip.fail('base transport stopped; last gripper target retained')
        if self.logic.enabled:
            commands,grip_packets=self.grip.tick(now,self.stationary(),self.motor_ready())
            if self.grip.phase=='ERROR':
                self.logic.stop(now,'gripper: '+self.grip.reason)
            else:
                for target in commands:
                    self.motor.publish(Float64MultiArray(data=[-1.,float(target),-1.,1.]))
            for packet in grip_packets:
                self.grip_wire.publish(String(data=json.dumps(packet,allow_nan=False)))
        description=self.grip.phase+': '+self.grip.reason
        self.grip_status.publish(String(data=description))
        if description!=self.last_grip_state:
            self.get_logger().info('GRIP '+description)
            self.last_grip_state=description
        value, packets = self.logic.step(now)
        if not self.logic.enabled and not self.released:
            self.release()
        if self.logic.enabled or now <= self.logic.stop_until:
            cmd = Twist()
            cmd.linear.x = value
            self.cmd.publish(cmd)
        for packet in packets:
            self.wire.publish(String(data=json.dumps(packet, allow_nan=False, separators=(',', ':'))))
        description = self.logic.state + ': ' + self.logic.reason
        self.status.publish(String(data=description))
        if description != self.last_state:
            self.get_logger().info(description)
            self.last_state = description

    def close(self):
        self.logic.stop(time.monotonic(), 'node shutdown', fault=False)
        for _ in range(3):
            self.publish_zero()
            self.wire.publish(String(data=json.dumps(self.logic.packet('STOP', reason='node shutdown'))))
        self.release()


def main(args=None):
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_args: stopped.set())
    # A signal sets a flag, including during construction. Keep the context
    # alive until zero/STOP publications and stop RPCs have been flushed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    node = None
    try:
        node = ManualStraightNode()
        executor.add_node(node)
        while rclpy.ok() and not stopped.is_set():
            executor.spin_once(timeout_sec=.05)
    finally:
        if node is not None:
            node.close()
            end = time.monotonic() + .2
            while rclpy.ok() and time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
