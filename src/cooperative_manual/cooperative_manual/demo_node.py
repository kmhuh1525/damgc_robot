"""Scenario signal -> paired grip -> leader-driven curve transport (no lift)."""
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from .curve_node import CurvedManualNode


class DemoTransportNode(CurvedManualNode):
    # Replaces the leader curve peer; never launch both nodes together.
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        if self.role!='leader':raise ValueError('demo coordinator runs on leader; follower uses manual_curve')
        for name,value in {'demo_command_topic':'/leader/demo/cmd_vel','demo_open_first':True,'demo_source_timeout':.25}.items():
            self.declare_parameter(name,value)
        timeout=float(self.get_parameter('demo_source_timeout').value)
        if not math.isfinite(timeout) or not 0<timeout<=1.:raise ValueError('demo source timeout must be in (0,1] seconds')
        self.stage='IDLE';self.stage_time=time.monotonic();self.last_source=None
        self.source_command=Twist();self.arm_future=None;self.arm_step=''
        self.demo_status=self.create_publisher(String,'/leader/demo_transport/status',10)
        self.remote_enable=self.create_client(SetBool,'/follower/manual_curve/enable')
        self.create_subscription(Twist,self.get_parameter('demo_command_topic').value,self.source,1)
        self.create_service(Trigger,'/leader/demo_transport/start',self.start_demo)
        self.create_service(Trigger,'/leader/demo_transport/cancel',self.cancel_demo)
        self.create_service(Trigger,'/leader/demo_transport/finish',self.finish_demo)
        self.report('IDLE','waiting for scenario start; robots must already be positioned at the object')

    @property
    def demo_active(self):return self.stage not in ('IDLE','DONE','CANCELLED','FAULT')

    def report(self,stage,reason):
        self.stage=stage;self.stage_time=time.monotonic()
        text=stage+': '+reason
        self.demo_status.publish(String(data=text));self.get_logger().info(text)

    def enable(self,request,response):
        if request.data and getattr(self,'stage','IDLE')!='ARM_LEADER':
            response.message='use /leader/demo_transport/start for the complete demo';return response
        response=super().enable(request,response)
        if not request.data and getattr(self,'demo_active',False):
            self.end_demo('CANCELLED','external stop')
        return response

    def manual(self,msg):
        # Keyboard transport topic must not bypass the scenario grip gate.
        if any(abs(x)>1e-8 for x in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)):
            if self.demo_active:self.end_demo('FAULT','manual curve keyboard conflicts with demo input')

    def start_demo(self,request,response):
        if self.demo_active or (self.arm_future is not None and not self.arm_future.done()):
            response.message='demo already running or previous arming request still pending';return response
        if not self.get_parameter('gripper_enabled').value or not self.get_parameter('motion_enabled').value:
            response.message='motion_enabled and gripper_enabled are required';return response
        if self.pose is None or not self.stationary() or not self.motor_ready():
            response.message='fresh stationary leader odometry and one gripper motor node required';return response
        if not self.remote_enable.service_is_ready():
            response.message='follower curve enable service unavailable';return response
        if self.count_publishers(self.transport_prefix+'/input'):
            response.message='stop curve keyboard before scenario mode';return response
        if self.count_publishers(self.command_topic)!=1 or self.count_publishers(self.transport_prefix+'/leader')!=1:
            response.message='another leader drive/curve controller is running';return response
        self.last_source=None;self.source_command=Twist();self.requested=0.
        self.logic.stop(time.monotonic(),'demo preparing',fault=False);self.release()
        self.report('ARM_FOLLOWER','reset and prepare follower with zero output')
        self.arm_step='disable'
        self.arm_future=self.remote_enable.call_async(SetBool.Request(data=False))
        response.success=True;response.message='accepted; watch demo status, no driving before paired GRASP DONE'
        return response

    def source(self,msg):
        if self.stage not in ('TRANSPORT_READY','TRANSPORTING'):return
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(v)>1e-6 for v in values[1:5]):
            self.end_demo('FAULT','invalid navigation command');return
        if self.count_publishers(self.get_parameter('demo_command_topic').value)!=1:
            self.end_demo('FAULT','conflicting navigation command publishers');return
        v,w=msg.linear.x,msg.angular.z
        # Leader pulls backward. The planner must avoid pivots/tight turns.
        if v>1e-6 or (abs(v)<1e-6 and abs(w)>1e-6) or abs(w)>abs(v)*self.max_curvature+1e-6:
            self.end_demo('FAULT','demo requires reverse-pull navigation, no pivots, curvature <=1/m');return
        scale=min(1.,self.logic.limits.speed/max(abs(v),1e-9))
        command=Twist();command.linear.x=v*scale;command.angular.z=w*scale
        self.source_command=command;self.last_source=time.monotonic()
        if self.stage=='TRANSPORT_READY':self.report('TRANSPORTING','fresh leader route commands accepted; follower tracks measured trail')

    def end_demo(self,stage,reason):
        self.requested=0.;self.source_command=Twist();self.last_source=None
        self.logic.stop(time.monotonic(),reason,fault=stage=='FAULT');self.release()
        if self.remote_enable.service_is_ready():self.remote_enable.call_async(SetBool.Request(data=False))
        self.report(stage,reason+'; grippers hold their last targets')

    def cancel_demo(self,request,response):
        self.end_demo('CANCELLED','scenario cancelled');response.success=True;response.message='both bases stopped; grasp retained';return response

    def finish_demo(self,request,response):
        if self.stage not in ('TRANSPORT_READY','TRANSPORTING'):
            response.message='transport is not ready/running';return response
        self.end_demo('DONE','route completed');response.success=True;response.message='both bases stopped; grasp retained';return response

    def late_arm(self,future):
        if not self.demo_active:
            try:
                if future.result().success and self.remote_enable.service_is_ready():
                    self.remote_enable.call_async(SetBool.Request(data=False))
            except Exception:pass

    def tick(self):
        if not hasattr(self,'stage'):return super().tick()
        now=time.monotonic()
        if self.demo_active:
            if self.stage=='ARM_FOLLOWER':
                if now-self.stage_time>5:self.end_demo('FAULT','follower arming service timeout')
                elif self.arm_future.done():
                    try:
                        result=self.arm_future.result()
                        if not result.success:raise ValueError(result.message)
                        if self.arm_step=='disable':
                            self.arm_step='enable';self.arm_future=self.remote_enable.call_async(SetBool.Request(data=True))
                            self.arm_future.add_done_callback(self.late_arm)
                        else:
                            self.report('ARM_LEADER','follower prepared; arming leader')
                            result=CurvedManualNode.enable(self,SetBool.Request(data=True),SetBool.Response())
                            if not result.success:raise ValueError(result.message)
                            self.report('WAIT_ACTIVE','waiting for both curve peers and fresh trail')
                    except Exception as exc:self.end_demo('FAULT','arming rejected: '+str(exc))
            elif self.logic.state in ('FAULT','DISABLED'):
                self.end_demo('FAULT','curve transport stopped: '+self.logic.reason)
            elif self.stage=='WAIT_ACTIVE' and self.logic.state=='ACTIVE':
                action='RELEASE' if self.get_parameter('demo_open_first').value else 'GRASP'
                self.grip_request(String(data=action))
                self.report('OPENING' if action=='RELEASE' else 'GRASPING','paired '+action+' requested')
            elif self.stage in ('OPENING','GRASPING'):
                if self.grip.phase=='ERROR':self.end_demo('FAULT','paired gripper: '+self.grip.reason)
                elif self.grip.phase=='DONE':
                    if self.stage=='OPENING':
                        self.grip_request(String(data='GRASP'));self.report('GRASPING','both opened; paired GRASP requested')
                    else:
                        self.last_source=None;self.source_command=Twist()
                        self.report('TRANSPORT_READY','both GRASP transactions completed; start leader reverse route now')
            if self.stage in ('TRANSPORT_READY','TRANSPORTING'):
                if self.last_source is not None and now-self.last_source>float(self.get_parameter('demo_source_timeout').value):
                    self.end_demo('FAULT','navigation command timeout')
                else:CurvedManualNode.manual(self,self.source_command)
            elif self.demo_active and self.logic.enabled:CurvedManualNode.manual(self,Twist())
        super().tick()


def main(args=None):
    # Reuse the curve runner's clean zero/STOP flushing on SIGINT/SIGTERM.
    import signal,threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    stopped=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopped.set())
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    executor=SingleThreadedExecutor();node=None
    try:
        node=DemoTransportNode();executor.add_node(node)
        while rclpy.ok() and not stopped.is_set():executor.spin_once(timeout_sec=.02)
    finally:
        if node is not None:
            node.end_demo('CANCELLED','demo node shutdown');node.close()
            end=time.monotonic()+.2
            while rclpy.ok() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
            executor.remove_node(node);node.destroy_node()
        executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
