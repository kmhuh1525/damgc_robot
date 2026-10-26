"""Real DDS/services without hardware, only in an explicitly isolated domain."""
import os
import time
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('COOPERATIVE_MANUAL_ISOLATED_TEST') != '1' or os.environ.get('ROS_DOMAIN_ID') != '176' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires explicit localhost-only ROS domain 176; never run on live robot domain',
)


def test_real_peer_handshake_shared_ramp_stop_and_latched_fault():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters, GetParameters
    from rcl_interfaces.msg import SetParametersResult, ParameterValue
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Float64MultiArray
    from nav_msgs.msg import Odometry
    from cooperative_manual.node import ManualStraightNode

    rclpy.init()
    controls = Node('manual_straight_test_controls')
    selection, guard = {}, {'enabled':False}
    outputs = {'leader':[], 'follower':[]}
    target = {'linear':0., 'turn':0., 'odom':True, 'reject_guard':False, 'reverse':True}
    services = []

    def select(request, response, role):
        for parameter in request.parameters:
            if parameter.name == 'source_mode':
                selection[role] = parameter.value.string_value
        response.results = [SetParametersResult(successful=True) for _ in request.parameters]
        return response

    def gate(request, response):
        if request.data and target['reject_guard']:
            response.success = False
            return response
        guard['enabled'] = request.data
        response.success = True
        return response

    for role in ('leader','follower'):
        services.append(controls.create_service(SetParameters, '/'+role+'/command_selector/set_parameters', lambda req,res,role=role:select(req,res,role)))
    services.append(controls.create_service(SetBool,'/follower/velocity_guard/enable',gate))
    def guard_config(request,response):
        response.values=[ParameterValue(type=1,bool_value=target['reverse'])]
        return response
    services.append(controls.create_service(GetParameters,'/follower/velocity_guard/get_parameters',guard_config))
    odom_l = controls.create_publisher(Odometry,'/leader/odometry/local',1)
    odom_f = controls.create_publisher(Odometry,'/follower/odom/raw',1)
    manual = controls.create_publisher(Twist,'/cooperation/manual/input',1)
    controls.create_subscription(Twist,'/leader/cooperation/cmd_vel',lambda m:outputs['leader'].append((m.linear.x,m.angular.z)),1)
    controls.create_subscription(Twist,'/follower/mission/cmd_vel',lambda m:outputs['follower'].append((m.linear.x,m.angular.z)),1)

    def stream():
        if target['odom']:
            msg = Odometry();msg.header.stamp = controls.get_clock().now().to_msg()
            odom_l.publish(msg);odom_f.publish(msg)
        cmd = Twist();cmd.linear.x = target['linear'];cmd.angular.z = target['turn']
        manual.publish(cmd)

    motor_commands={'leader':[],'follower':[]}
    motor_status={role:controls.create_publisher(String,'/'+role+'/dynamixel/status',10) for role in ('leader','follower')}
    def motor(message,role):
        assert message.data[0]==-1. and list(message.data[2:])==[-1.,1.]
        motor_commands[role].append(message.data[1])
        motor_status[role].publish(String(data='OK rx64=-1 rx28=%d rx64_torque=-1 rx28_torque=1'%message.data[1]))
    for role in ('leader','follower'):
        controls.create_subscription(Float64MultiArray,'/'+role+'/dynamixel/command',lambda m,role=role:motor(m,role),10)
    grip_request=controls.create_publisher(String,'/cooperation/manual/gripper/request',10)
    timer = controls.create_timer(.02,stream)
    leader = ManualStraightNode(namespace='/leader',parameter_overrides=[Parameter('role',value='leader'),Parameter('motion_enabled',value=True),Parameter('gripper_enabled',value=True),Parameter('close_raw',value=480),Parameter('gripper_settle',value=.2)])
    follower = ManualStraightNode(namespace='/follower',parameter_overrides=[Parameter('role',value='follower'),Parameter('motion_enabled',value=True),Parameter('gripper_enabled',value=True),Parameter('gripper_settle',value=.1)])
    executor = SingleThreadedExecutor()
    for node in (controls,leader,follower):executor.add_node(node)
    c_l = controls.create_client(SetBool,'/leader/manual_straight/enable')
    c_f = controls.create_client(SetBool,'/follower/manual_straight/enable')

    def until(predicate,timeout=4.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert predicate(), (leader.logic.state,leader.logic.reason,follower.logic.state,follower.logic.reason)

    def enable(client,value):
        future = client.call_async(SetBool.Request(data=value))
        until(future.done)
        result = future.result()
        assert result.success,result.message

    try:
        until(lambda:c_l.service_is_ready() and c_f.service_is_ready() and leader.odom_time is not None and follower.odom_time is not None)
        assert all(name in ('manual_straight','manual_straight_test_controls') for name in controls.get_node_names()),'isolated domain has unexpected nodes'
        enable(c_f,True);enable(c_l,True)
        until(lambda:leader.logic.state==follower.logic.state=='ACTIVE')
        assert leader.logic.value==follower.logic.value==0.
        target['linear']=-.03
        until(lambda:leader.logic.value<-.02 and follower.logic.value>.02)
        assert selection=={'leader':'COOPERATION','follower':'COOPERATION'} and guard['enabled']
        assert all(v<=0 and w==0 for v,w in outputs['leader'])
        assert all(v>=0 and w==0 for v,w in outputs['follower'])
        until(lambda:leader.motor_ready() and follower.motor_ready())
        grip_request.publish(String(data='GRASP'))
        until(lambda:leader.grip.phase==follower.grip.phase=='DONE')
        assert motor_commands=={'leader':[480.],'follower':[331.]}
        assert leader.logic.value==follower.logic.value==0.
        assert leader.need_neutral  # Holding a drive key cannot resume motion.
        grip_request.publish(String(data='RELEASE'))
        until(lambda:leader.grip.action=='RELEASE' and leader.grip.phase==follower.grip.phase=='DONE')
        assert motor_commands=={'leader':[480.,1000.],'follower':[331.,1000.]}
        target['linear']=0.
        until(lambda:leader.logic.value==follower.logic.value==0. and not leader.need_neutral)
        target['linear']=.03
        until(lambda:leader.logic.value>.015 and follower.logic.value<-.015)
        target['linear']=0.
        until(lambda:leader.logic.value==follower.logic.value==0. and not leader.need_neutral)
        target['linear']=-.03
        until(lambda:leader.logic.value<-.015 and follower.logic.value>.015)
        enable(c_l,False)
        until(lambda:follower.logic.state=='FAULT' and not guard['enabled'])
        assert leader.logic.value==follower.logic.value==0.
        until(lambda:selection=={'leader':'STOP','follower':'STOP'})
        target['linear']=0.
        enable(c_f,True);enable(c_l,True)
        until(lambda:leader.logic.state==follower.logic.state=='ACTIVE')
        target['turn']=.1
        until(lambda:leader.logic.state==follower.logic.state=='FAULT')
        assert leader.logic.value==follower.logic.value==0.
        target['turn']=0.
        enable(c_f,True);enable(c_l,True)
        until(lambda:leader.logic.state==follower.logic.state=='ACTIVE')
        target['odom']=False
        until(lambda:leader.logic.state==follower.logic.state=='FAULT')
        assert leader.logic.value==follower.logic.value==0.
        target['odom']=True
        until(lambda:follower.odom_time is not None and time.monotonic()-follower.odom_time<.1)
        target['reject_guard']=True
        enable(c_f,True)
        until(lambda:follower.logic.state=='FAULT')
        assert 'rejected' in follower.logic.reason
        assert follower.logic.value==0.
        target['reject_guard']=False
        target['reverse']=False
        enable(c_f,True)
        until(lambda:follower.logic.state=='FAULT')
        assert 'reverse-capable' in follower.logic.reason
    finally:
        for node in (leader,follower):node.close()
        end=time.monotonic()+.1
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        for node in (leader,follower,controls):
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()


@pytest.mark.parametrize("signal_name", ["SIGINT", "SIGTERM"])
def test_cli_shutdown_flushes_stop_before_context_shutdown(signal_name):
    import subprocess
    import sys
    import signal
    child = subprocess.Popen([sys.executable, '-u', '-c', 'from cooperative_manual.node import main; main()'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        import select
        ready = False
        end = time.monotonic()+8
        while time.monotonic()<end and child.poll() is None:
            if select.select([child.stdout],[],[],.1)[0]:
                line=child.stdout.readline()
                if 'explicit enable required' in line:
                    ready=True
                    break
        assert ready, 'standalone disabled node did not start'
        child.send_signal(getattr(signal, signal_name))
        output,_=child.communicate(timeout=4)
        assert child.returncode==0,output
        assert 'Traceback' not in output
    finally:
        if child.poll() is None:
            child.kill();child.communicate()
