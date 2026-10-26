"""Opt-in live leader-odometry curve follower; existing straight mode stays default."""
import json
import math
import signal
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from cooperative_transport.motion import Pose2D,normalize_angle
from cooperative_transport.hinged_formation import HingeGeometry
from .curve import LeaderTrail,FollowerTrail,CurveOutput,leader_scale,FollowerSmoother
from .node import ManualStraightNode


class CurvedManualNode(ManualStraightNode):
    NODE_NAME='manual_curve'
    TRANSPORT_PREFIX='/cooperation/curve'
    ENABLE_SERVICE='manual_curve/enable'
    DEFAULT_SPEED=.02
    DEFAULT_ACCELERATION=.02

    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        for name,value in {'hinge_limit_deg':60.,'axle_to_hinge':.125,'hinge_to_contact':.1325,'object_center_to_contact':.0525,'max_leader_curvature':1.0}.items():self.declare_parameter(name,value)
        value=lambda name:float(self.get_parameter(name).value)
        self.geometry=HingeGeometry(value('axle_to_hinge'),value('hinge_to_contact'),value('object_center_to_contact'),math.radians(value('hinge_limit_deg')))
        self.geometry.validate()
        self.max_curvature=value('max_leader_curvature')
        if not 0<self.max_curvature<=1.:raise ValueError('curve leader curvature must be in (0,1]')
        q=self.geometry.hinge_limit*.8
        physical=math.sin(q)/(self.geometry.axle_to_hinge*math.cos(q)+self.geometry.longitudinal_offset)
        self.max_curvature=min(self.max_curvature,physical)
        self.pose=None;self.frame=None;self.initial_pose=None
        self.trail=LeaderTrail(self.geometry)
        self.follow=FollowerTrail(self.geometry)
        self.trail_session=''
        self.observed_linear=0.;self.peer_observed_linear=0.
        self.requested=0.;self.curvature_target=0.;self.curvature=0.;self.scale=1.
        self.smoother=FollowerSmoother();self.output_time=time.monotonic()
        self.feedback=CurveOutput(0.,0.)
        self.last_curve_tick=time.monotonic();self.last_trail_send=0.
        self.trail_wire=self.create_publisher(String,'/cooperation/curve/trail',1) if self.role=='leader' else None
        if self.role=='follower':self.create_subscription(String,'/cooperation/curve/trail',self.receive_trail,1)
        self.path_pub=self.create_publisher(Path,'/cooperation/curve/'+self.role+'/path',1)
        self.get_logger().info('CURVE mode: measured trail, hinge60deg, soft tracking corrections; no tracking-error FAULT. Motion remains opt-in.')

    def odometry(self,msg):
        super().odometry(msg)
        if not hasattr(self,'trail'):return
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        values=(p.x,p.y,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(x) for x in values):return
        norm=sum(x*x for x in values[2:])**.5
        if norm<1e-9:return
        x,y,z,w=(v/norm for v in values[2:])
        pose=Pose2D(p.x,p.y,math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not 0<=self.get_clock().now().nanoseconds/1e9-stamp<=.5:return
        if self.logic.enabled and self.frame is not None and self.frame!=msg.header.frame_id:
            self.logic.stop(time.monotonic(),'odometry frame changed');return
        self.pose=pose;self.frame=msg.header.frame_id
        self.observed_linear=float(msg.twist.twist.linear.x)
        if self.role=='leader' and self.logic.enabled and self.trail.origin is not None:
            try:self.trail.add(pose)
            except ValueError as exc:self.logic.stop(time.monotonic(),str(exc))

    def enable(self,request,response):
        if request.data and self.pose is None:
            response.message='fresh pose odometry required for curve mode';return response
        response=super().enable(request,response)
        if request.data and response.success:
            self.smoother.reset();self.output_time=time.monotonic()
            self.initial_pose=self.pose;self.requested=0.;self.scale=1.;self.curvature=self.curvature_target=0.
            self.trail_session=self.logic.session
            if self.role=='leader':self.trail.begin(self.pose)
            else:self.follow.reset(self.initial_pose)
        return response

    def manual(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not self.logic.enabled:return
        if not all(math.isfinite(x) for x in values) or any(abs(x)>1e-6 for x in values[1:5]):
            self.logic.stop(time.monotonic(),'curve mode accepts longitudinal motion and yaw only');return
        if msg.linear.x>0 and abs(msg.angular.z)>1e-6:
            self.logic.stop(time.monotonic(),'backward return is straight only');return
        if abs(msg.linear.x)<.001 and abs(msg.angular.z)>1e-6:
            self.logic.stop(time.monotonic(),'in-place rotation forbidden while coupled');return
        self.curvature_target=max(-self.max_curvature,min(self.max_curvature,msg.angular.z/msg.linear.x)) if abs(msg.linear.x)>=.001 else 0.
        straight=Twist();straight.linear.x=msg.linear.x
        super().manual(straight)
        self.requested=self.logic.target

    def receive(self,msg):
        try:
            packet=json.loads(msg.data)
            if self.role=='leader' and packet.get('kind')=='STATUS':
                if packet.get('session')!=self.logic.session or not packet.get('curve_ready'):return
                self.scale=leader_scale(float(packet.get('lag',0.)),float(packet.get('cross_track',0.)))
            if self.role=='follower' and packet.get('kind')=='COMMAND':
                v=packet.get('observed_linear')
                if not isinstance(v,(float,int)) or not math.isfinite(v):raise ValueError('missing observed leader speed')
                if packet.get('session')==self.logic.session:
                    self.peer_observed_linear=max(-.05,min(.05,float(v)))
            super().receive(msg)
            if self.role=='follower' and self.logic.session!=self.trail_session:
                self.trail_session=self.logic.session;self.follow.reset(self.initial_pose)
        except (ValueError,TypeError,AttributeError) as exc:
            if self.logic.enabled:self.logic.stop(time.monotonic(),'invalid curve feedback: '+str(exc))

    def decorate_packet(self,packet):
        if packet['kind']=='STATUS':
            packet.update(curve_ready=self.follow.ready,lag=self.feedback.lag,cross_track=self.feedback.cross_track)
        elif packet['kind']=='COMMAND':packet['observed_linear']=self.observed_linear
        return packet

    def receive_trail(self,msg):
        if not self.logic.enabled or len(msg.data)>196608:return
        try:
            packet=json.loads(msg.data)
            if packet.get('session')!=self.logic.session or not self.logic.session:return
            self.follow.receive(packet,time.monotonic())
            self.publish_path(self.follow.path)
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.logic.stop(time.monotonic(),'invalid curve trail: '+str(exc))

    def publish_path(self,poses):
        path=Path();path.header.frame_id=self.frame or 'odom';path.header.stamp=self.get_clock().now().to_msg()
        for pose in poses:
            ps=PoseStamped();ps.header=path.header;ps.pose.position.x=pose.x;ps.pose.position.y=pose.y
            ps.pose.orientation.z=math.sin(pose.yaw/2);ps.pose.orientation.w=math.cos(pose.yaw/2)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def prepare_command(self,value,now):
        command=Twist()
        dt=now-self.output_time;self.output_time=now
        if abs(value)<1e-8:
            self.smoother.reset()
            return command
        if self.role=='leader':
            command.linear.x=value;command.angular.z=value*self.curvature
        else:
            # Follow measured motion, not a fictitious desired moving leader.
            self.feedback=self.smoother.step(value,self.peer_observed_linear,dt,
                lambda observed:self.follow.output(self.pose,observed,now))
            command.linear.x=self.feedback.linear;command.angular.z=self.feedback.angular
        return command

    def tick(self):
        if not hasattr(self,'trail'):return super().tick()
        now=time.monotonic();dt=min(.1,max(0.,now-self.last_curve_tick));self.last_curve_tick=now
        if self.role=='leader' and self.logic.enabled:
            self.logic.target=0. if self.grip.busy or self.need_neutral else self.requested*self.scale
            target=0. if self.grip.busy else self.curvature_target
            self.curvature+=max(-.25*dt,min(.25*dt,target-self.curvature))
            if self.trail.origin is not None and now-self.last_trail_send>=.1:
                self.last_trail_send=now
                self.trail_wire.publish(String(data=json.dumps(self.trail.packet(self.logic.session),allow_nan=False)))
                self.publish_path([Pose2D(*p[:3]) for p in self.trail.points])
        super().tick()


def main(args=None):
    stopped=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopped.set())
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    executor=SingleThreadedExecutor();node=None
    try:
        node=CurvedManualNode();executor.add_node(node)
        while rclpy.ok() and not stopped.is_set():executor.spin_once(timeout_sec=.03)
    finally:
        if node is not None:
            node.close();end=time.monotonic()+.2
            while rclpy.ok() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
            executor.remove_node(node);node.destroy_node()
        executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
