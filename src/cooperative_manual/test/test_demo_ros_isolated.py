"""Closed ROS loop with simulated unicycles, never a real bridge."""
import os
import math
import time
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='176' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('COOPERATIVE_MANUAL_ISOLATED_TEST')!='1',reason='explicit isolated domain176 required')


@pytest.mark.parametrize('failure',['normal','missing_ack','cancel_arming','invalid_navigation'])
def test_scenario_grip_gate_route_stream_and_source_timeout(failure):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters,GetParameters
    from rcl_interfaces.msg import SetParametersResult,ParameterValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool
    from cooperative_manual.curve_node import CurvedManualNode
    from cooperative_manual.demo_node import DemoTransportNode
    from std_msgs.msg import String,Float64MultiArray
    from std_srvs.srv import Trigger
    rclpy.init();control=Node('curve_test_controls');executor=SingleThreadedExecutor()
    drive={'leader':(0.,0.),'follower':(0.,0.)};poses={'leader':[0.,0.,0.],'follower':[5.,-2.,math.pi/2]};gate={'on':False};target={'v':0.,'w':0.,'send':True};seen={'leader':[],'follower':[]}
    def select(req,res):res.results=[SetParametersResult(successful=True) for _ in req.parameters];return res
    def guard(req,res):gate['on']=req.data;res.success=True;return res
    def config(req,res):res.values=[ParameterValue(type=1,bool_value=True)];return res
    for role in ('leader','follower'):control.create_service(SetParameters,'/'+role+'/command_selector/set_parameters',select)
    control.create_service(SetBool,'/follower/velocity_guard/enable',guard)
    control.create_service(GetParameters,'/follower/velocity_guard/get_parameters',config)
    pubs={r:control.create_publisher(Odometry,'/leader/odometry/local' if r=='leader' else '/follower/odom/raw',1) for r in ('leader','follower')}
    command=control.create_publisher(Twist,'/leader/demo/cmd_vel',1)
    def capture(msg,role):drive[role]=(msg.linear.x,msg.angular.z);seen[role].append(drive[role])
    for role,topic in [('leader','/leader/cooperation/cmd_vel'),('follower','/follower/mission/cmd_vel')]:control.create_subscription(Twist,topic,lambda m,role=role:capture(m,role),1)
    def plant():
        for role in ('leader','follower'):
            v,w=drive[role] if role=='leader' or gate['on'] else (0.,0.)
            p=poses[role];p[0]+=v*math.cos(p[2]+w*.01)*.02;p[1]+=v*math.sin(p[2]+w*.01)*.02;p[2]+=w*.02
            m=Odometry();m.header.stamp=control.get_clock().now().to_msg();m.header.frame_id=role+'_odom'
            m.pose.pose.position.x=p[0];m.pose.pose.position.y=p[1];m.pose.pose.orientation.z=math.sin(p[2]/2);m.pose.pose.orientation.w=math.cos(p[2]/2);m.twist.twist.linear.x=v;m.twist.twist.angular.z=w;pubs[role].publish(m)
        msg=Twist();msg.linear.x=target['v'];msg.angular.z=target['w'];
        if target['send']:command.publish(msg)
    control.create_timer(.02,plant)
    leader=DemoTransportNode(namespace='/leader',parameter_overrides=[Parameter('role',value='leader'),Parameter('motion_enabled',value=True),Parameter('gripper_enabled',value=True),Parameter('close_raw',value=477),Parameter('gripper_settle',value=.06)])
    follower=CurvedManualNode(namespace='/follower',parameter_overrides=[Parameter('role',value='follower'),Parameter('motion_enabled',value=True),Parameter('gripper_enabled',value=True),Parameter('gripper_settle',value=.06)])
    for n in (control,leader,follower):executor.add_node(n)
    motors={'leader':[],'follower':[]}
    for role in ('leader','follower'):
        status=control.create_publisher(String,'/'+role+'/dynamixel/status',10)
        def motor(msg,role=role,status=status):
            value=int(msg.data[1]);motors[role].append(value)
            if failure!='missing_ack':status.publish(String(data='OK rx28=%d '%value))
        control.create_subscription(Float64MultiArray,'/'+role+'/dynamixel/command',motor,10)
    def until(test,seconds=5.):
        end=time.monotonic()+seconds
        while not test() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert test(),(leader.logic.state,leader.logic.reason,follower.logic.state,follower.logic.reason)
    try:
        until(lambda:leader.pose is not None and follower.pose is not None and leader.motor_ready())
        target.update(v=-.02,w=.01)  # Commands before grip must never start either base.
        start=control.create_client(Trigger,'/leader/demo_transport/start');until(start.service_is_ready)
        future=start.call_async(Trigger.Request());until(future.done);assert future.result().success
        if failure=='cancel_arming':
            cancel=control.create_client(Trigger,'/leader/demo_transport/cancel');until(cancel.service_is_ready)
            f=cancel.call_async(Trigger.Request());until(f.done);assert f.result().success
            end=time.monotonic()+.6
            while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
            assert leader.stage=='CANCELLED' and not follower.logic.enabled
            assert all(abs(v)<1e-8 for v,w in seen['leader'])
            return
        if failure=='missing_ack':
            until(lambda:leader.stage=='FAULT',5.)
            until(lambda:not follower.logic.enabled)
            assert all(abs(v)<1e-8 for v,w in seen['leader'])
            assert 477 not in motors['leader'] and 331 not in motors['follower']
            return
        repeated=start.call_async(Trigger.Request());until(repeated.done);assert not repeated.result().success
        until(lambda:leader.stage in ('TRANSPORT_READY','TRANSPORTING'))
        assert motors['leader']==[1000,477] and motors['follower']==[1000,331]
        assert all(abs(v)<1e-8 for v,w in seen['leader'])
        until(lambda:leader.stage=='TRANSPORTING' and any(v<-.004 for v,w in seen['leader']) and any(v>.001 for v,w in seen['follower']))
        if failure=='invalid_navigation':
            target.update(v=.02,w=.01)
            until(lambda:leader.stage=='FAULT' and not follower.logic.enabled)
            assert 'reverse' in leader.logic.reason
            return
        target.update(v=0.,w=0.)
        until(lambda:abs(drive['leader'][0])<1e-6 and abs(drive['follower'][0])<1e-6)
        finish=control.create_client(Trigger,'/leader/demo_transport/finish');until(finish.service_is_ready)
        done=finish.call_async(Trigger.Request());until(done.done);assert done.result().success
        until(lambda:leader.stage=='DONE' and not follower.logic.enabled)
        assert motors['leader']==[1000,477] and motors['follower']==[1000,331]
        # A new accepted cycle must not reuse the old paired-grip DONE result.
        target.update(v=0.,w=0.)
        future=start.call_async(Trigger.Request());until(future.done);assert future.result().success
        until(lambda:leader.stage=='TRANSPORTING' and len(motors['leader'])==4)
        target.update(v=-.02,w=.01)
        until(lambda:abs(drive['leader'][0])>.002)
        target['send']=False
        until(lambda:leader.stage=='FAULT' and not follower.logic.enabled,3.)
        assert 'timeout' in leader.logic.reason
    finally:
        leader.close();follower.close()
        end=time.monotonic()+.1
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        for n in (leader,follower,control):executor.remove_node(n);n.destroy_node()
        executor.shutdown();rclpy.shutdown()
