"""Dedicated straight drive + paired grip keys. Does not select TELEOP."""
import os
import select
import sys
import termios
import threading
import time
import tty
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main(args=None, opening=False):
    if not sys.stdin.isatty():
        raise RuntimeError('interactive terminal required')
    stopped=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopped.set())
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('opening_ceremony_keys' if opening else 'manual_straight_keys')
    pub=node.create_publisher(Twist,'/cooperation/manual/input',1)
    grip=node.create_publisher(String,'/cooperation/manual/gripper/request',10)
    enable=node.create_client(SetBool,'/leader/manual_straight/enable')
    last_status={}
    def report(label,msg):
        if msg.data!=last_status.get(label):
            print('\r\n'+label+' '+msg.data,flush=True);last_status[label]=msg.data
    node.create_subscription(String,'/cooperation/manual/leader/status',lambda m:report('DRIVE',m),1)
    node.create_subscription(String,'/cooperation/manual/gripper/leader/status',lambda m:report('GRIP',m),10)
    pending=[]
    def control(value):
        if enable.service_is_ready():pending.append(enable.call_async(SetBool.Request(data=value)))
        else:print('\r\nleader enable service unavailable',flush=True)
    if opening:
        print('개회식: E=준비; R=양쪽 열기; G=양쪽 파지; GRIP DONE 확인 후 ↑/W 한 번=계속 직진(리더 후진·서브 전진); ↓/S=직선 복귀; X=감속 정지; SPACE=비상정지; Q=종료. 키를 놓아도 계속 이동. 회전·리프트 없음.',flush=True)
    else:print('e=enable; HOLD UP/w=leader forward, DOWN/s=leader reverse (follower opposite); release/x=gentle stop; G=both grasp; R=both release; SPACE=abort bases; q=quit. No turns.',flush=True)
    old=termios.tcgetattr(sys.stdin.fileno())
    last_hold,last_publish,direction=-10.,0.,0
    buffer=''
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node,timeout_sec=.01)
            for future in pending[:]:
                if future.done():print('\r\n'+str(future.result()),flush=True);pending.remove(future)
            if select.select([sys.stdin],[],[],0)[0]:
                buffer+=os.read(sys.stdin.fileno(),32).decode('latin1')
                while buffer:
                    if buffer.startswith('\x1b'):
                        if buffer in ('\x1b','\x1b['):break
                        if buffer.startswith(('\x1b[A','\x1b[B','\x1b[C','\x1b[D')):
                            key={'A':'w','B':'s','C':'x','D':'x'}[buffer[2]];buffer=buffer[3:]
                        else:key='x';buffer=buffer[1:]
                    else:key,buffer=buffer[0].lower(),buffer[1:]
                    if key in ('w','s'):
                        direction=(1 if key=='w' else -1)*(-1 if opening else 1);last_hold=time.monotonic()
                    elif key=='e':direction=0;control(True)
                    elif key in ('g','r'):
                        direction=0;pub.publish(Twist())
                        grip.publish(String(data='GRASP' if key=='g' else 'RELEASE'))
                    elif key in (' ','q','\x03'):
                        direction=0;pub.publish(Twist());control(False)
                        if key in ('q','\x03'):return
                    elif key in ('x','a','d'):direction=0
            now=time.monotonic()
            if now-last_publish>=.05:
                command=Twist()
                if opening or now-last_hold<=.25:command.linear.x=direction*.03
                pub.publish(command);last_publish=now
    finally:
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old)
        if rclpy.ok():
            pub.publish(Twist());control(False)
            end=time.monotonic()+.2
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def opening_main(args=None):
    main(args, opening=True)
