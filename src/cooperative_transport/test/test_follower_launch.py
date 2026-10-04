"""Exercise installed launch defaults with physical bridges excluded."""
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def test_installed_follower_launch_stays_idle_with_zero_output(tmp_path):
    log = (tmp_path/'launch.log').open('w+')
    process = subprocess.Popen([
        'ros2', 'launch', 'cooperative_transport', 'manual_transport.launch.py',
        'role:=follower', 'follower_drive:=true', 'use_stm32_bridge:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    observer = Node('launch_observer')
    outputs, states = [], []
    observer.create_subscription(Twist, '/follower/safe_cmd_vel',
                                 lambda m: outputs.append((m.linear.x,m.angular.z)), 10)
    observer.create_subscription(String, '/cooperation/transport/follower/status',
                                 lambda m: states.append(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic()+10.
        while (not outputs or not states) and time.monotonic() < deadline:
            assert process.poll() is None, 'follower launch exited'
            rclpy.spin_once(observer, timeout_sec=.05)
        assert outputs and all(v == (0.,0.) for v in outputs)
        assert states and ' IDLE:' in states[-1]
        for name, params, expected in [
            ('transport_peer', ['motion_enabled'], [False]),
            ('command_selector', ['source_mode'], ['STOP']),
            ('velocity_guard', ['allow_reverse','guard_enabled_on_startup'], [True,False]),
        ]:
            client = observer.create_client(GetParameters, '/follower/'+name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(GetParameters.Request(names=params))
            rclpy.spin_until_future_complete(observer, future, timeout_sec=2.)
            assert future.done()
            values = [p.string_value if p.type == 4 else p.bool_value for p in future.result().values]
            assert values == expected
        for topic in ('/follower/mission/cmd_vel','/follower/selected_cmd_vel','/follower/safe_cmd_vel'):
            assert observer.count_publishers(topic) == 1
        assert observer.count_subscribers('/follower/mission/cmd_vel') == 1
        assert observer.count_subscribers('/follower/selected_cmd_vel') == 1
        names = [name for name,_ in observer.get_node_names_and_namespaces()]
        assert not any('stm32' in name or 'dynamixel' in name or 'mission_executor' in name for name in names)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        observer.destroy_node()
        rclpy.shutdown()
        log.close()
