"""Exercise the real ceremony terminal using a pseudo-TTY; no robot nodes."""
import os
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='176' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('COOPERATIVE_MANUAL_ISOLATED_TEST')!='1',reason='isolated localhost domain176 only')


def test_one_key_cruise_stop_grip_and_abort():
    import pty,subprocess,sys,time
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    rclpy.init();node=Node('ceremony_terminal_test');commands=[];grips=[];requests=[]
    node.create_subscription(Twist,'/cooperation/manual/input',lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    node.create_subscription(String,'/cooperation/manual/gripper/request',lambda m:grips.append(m.data),10)
    def enable(req,res):requests.append(req.data);res.success=True;return res
    node.create_service(SetBool,'/leader/manual_straight/enable',enable)
    master,slave=pty.openpty()
    process=subprocess.Popen([sys.executable,'-c','from cooperative_manual.keyboard import opening_main; opening_main()'],stdin=slave,stdout=slave,stderr=slave)
    os.close(slave)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    try:
        end=time.monotonic()+5
        while not commands and time.monotonic()<end:spin(.05)
        assert commands,'keyboard failed to publish'
        os.write(master,b'e');spin(.2);assert True in requests
        os.write(master,b'\x1b[A');spin(.9)
        assert commands[-1][1]==-.03 and commands[-1][2]==0.
        assert all(v==-.03 for t,v,w in commands if t>time.monotonic()-.4)
        os.write(master,b'x');spin(.2);assert commands[-1][1]==0.
        os.write(master,b'\x1b[B');spin(.2);assert commands[-1][1]==.03
        os.write(master,b'g');spin(.3);assert grips[-1]=='GRASP' and commands[-1][1]==0.
        os.write(master,b'r');spin(.3);assert grips[-1]=='RELEASE'
        os.write(master,b' ');spin(.3);assert requests[-1] is False and commands[-1][1]==0.
        os.write(master,b'q');spin(.4);process.wait(timeout=3);assert process.returncode==0
    finally:
        if process.poll() is None:process.terminate();process.wait(timeout=3)
        os.close(master);node.destroy_node();rclpy.shutdown()
