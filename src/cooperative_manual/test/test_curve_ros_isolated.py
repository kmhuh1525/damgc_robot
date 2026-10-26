"""Closed ROS loop with simulated unicycles, never a real bridge."""
import os
import math
import time
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='176' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('COOPERATIVE_MANUAL_ISOLATED_TEST')!='1',reason='explicit isolated domain176 required')


def test_curve_handshake_motion_soft_error_and_missing_trail_stop():
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
    rclpy.init();control=Node('curve_test_controls');executor=SingleThreadedExecutor()
    drive={'leader':(0.,0.),'follower':(0.,0.)};poses={'leader':[0.,0.,0.],'follower':[5.,-2.,math.pi/2]};gate={'on':False};target={'v':0.,'w':0.};seen={'leader':[],'follower':[]}
    def select(req,res):res.results=[SetParametersResult(successful=True) for _ in req.parameters];return res
    def guard(req,res):gate['on']=req.data;res.success=True;return res
    def config(req,res):res.values=[ParameterValue(type=1,bool_value=True)];return res
    for role in ('leader','follower'):control.create_service(SetParameters,'/'+role+'/command_selector/set_parameters',select)
    control.create_service(SetBool,'/follower/velocity_guard/enable',guard)
    control.create_service(GetParameters,'/follower/velocity_guard/get_parameters',config)
    pubs={r:control.create_publisher(Odometry,'/leader/odometry/local' if r=='leader' else '/follower/odom/raw',1) for r in ('leader','follower')}
    command=control.create_publisher(Twist,'/cooperation/curve/input',1)
    def capture(msg,role):drive[role]=(msg.linear.x,msg.angular.z);seen[role].append(drive[role])
    for role,topic in [('leader','/leader/cooperation/cmd_vel'),('follower','/follower/mission/cmd_vel')]:control.create_subscription(Twist,topic,lambda m,role=role:capture(m,role),1)
    def plant():
        for role in ('leader','follower'):
            v,w=drive[role] if role=='leader' or gate['on'] else (0.,0.)
            p=poses[role];p[0]+=v*math.cos(p[2]+w*.01)*.02;p[1]+=v*math.sin(p[2]+w*.01)*.02;p[2]+=w*.02
            m=Odometry();m.header.stamp=control.get_clock().now().to_msg();m.header.frame_id=role+'_odom'
            m.pose.pose.position.x=p[0];m.pose.pose.position.y=p[1];m.pose.pose.orientation.z=math.sin(p[2]/2);m.pose.pose.orientation.w=math.cos(p[2]/2);m.twist.twist.linear.x=v;m.twist.twist.angular.z=w;pubs[role].publish(m)
        msg=Twist();msg.linear.x=target['v'];msg.angular.z=target['w'];command.publish(msg)
    control.create_timer(.02,plant)
    leader=CurvedManualNode(namespace='/leader',parameter_overrides=[Parameter('role',value='leader'),Parameter('motion_enabled',value=True)])
    follower=CurvedManualNode(namespace='/follower',parameter_overrides=[Parameter('role',value='follower'),Parameter('motion_enabled',value=True)])
    for n in (control,leader,follower):executor.add_node(n)
    def until(test,seconds=5.):
        end=time.monotonic()+seconds
        while not test() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert test(),(leader.logic.state,leader.logic.reason,follower.logic.state,follower.logic.reason)
    def enable(role):
        c=control.create_client(SetBool,'/'+role+'/manual_curve/enable');until(c.service_is_ready)
        f=c.call_async(SetBool.Request(data=True));until(f.done);assert f.result().success,f.result().message
    try:
        until(lambda:leader.pose is not None and follower.pose is not None)
        enable('follower');enable('leader');until(lambda:leader.logic.state==follower.logic.state=='ACTIVE')
        assert follower.follow.ready
        target.update(v=-.02,w=.012)
        until(lambda:any(v<-.005 and abs(w)>1e-5 for v,w in seen['leader']) and any(v>.001 and abs(w)>1e-6 for v,w in seen['follower']))
        poses['follower'][0]+=.12
        until(lambda:follower.feedback.position_error>.08 and leader.scale<.8)
        assert leader.logic.state==follower.logic.state=='ACTIVE'
        assert all(abs(w)<=.080001 and abs(v)<=.040001 for v,w in seen['follower'])
        leader.last_trail_send=time.monotonic()+5
        until(lambda:follower.logic.state=='FAULT' and leader.logic.state=='FAULT',3.)
        assert 'trail' in follower.logic.reason or 'peer' in follower.logic.reason
        until(lambda:not gate['on'])
    finally:
        leader.close();follower.close()
        end=time.monotonic()+.1
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        for n in (leader,follower,control):executor.remove_node(n);n.destroy_node()
        executor.shutdown();rclpy.shutdown()
