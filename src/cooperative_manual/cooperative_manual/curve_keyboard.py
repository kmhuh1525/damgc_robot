"""Experimental reverse-pull curve keys; never starts without explicit enable."""
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main(args=None):
    if not sys.stdin.isatty():raise RuntimeError('interactive terminal required')
    stopped=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopped.set())
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('manual_curve_keys');pub=node.create_publisher(Twist,'/cooperation/curve/input',1)
    grip=node.create_publisher(String,'/cooperation/manual/gripper/request',10)
    enable=node.create_client(SetBool,'/leader/manual_curve/enable');pending=[];last={}
    def report(label,msg):
        if last.get(label)!=msg.data:print('\r\n'+label+' '+msg.data,flush=True);last[label]=msg.data
    node.create_subscription(String,'/cooperation/curve/leader/status',lambda m:report('CURVE',m),1)
    node.create_subscription(String,'/cooperation/manual/gripper/leader/status',lambda m:report('GRIP',m),10)
    def control(value):
        if enable.service_is_ready():pending.append(enable.call_async(SetBool.Request(data=value)))
        else:print('\r\ncurve enable service unavailable',flush=True)
    print('EXPERIMENTAL CURVE: e=enable after follower; hold w/UP=straight reverse pull; hold a/LEFT,d/RIGHT=reverse curve (one key); hold s/DOWN=backward straight; x=stop; G/R=paired grip/release; SPACE=abort; q=exit. No in-place turns.',flush=True)
    old=termios.tcgetattr(sys.stdin.fileno());buffer='';moving=False;direction=-1.;last_move=-10.;turn=0.;last_turn=-10.;last_publish=0.
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node,timeout_sec=.01)
            for f in pending[:]:
                if f.done():print('\r\n'+str(f.result()),flush=True);pending.remove(f)
            if select.select([sys.stdin],[],[],0)[0]:
                buffer+=os.read(sys.stdin.fileno(),32).decode('latin1')
                while buffer:
                    if buffer.startswith('\x1b'):
                        if buffer in ('\x1b','\x1b['):break
                        if buffer.startswith(('\x1b[A','\x1b[B','\x1b[C','\x1b[D')):
                            key={'A':'w','B':'s','C':'d','D':'a'}[buffer[2]];buffer=buffer[3:]
                        else:key='x';buffer=buffer[1:]
                    else:key,buffer=buffer[0].lower(),buffer[1:]
                    now=time.monotonic()
                    if key=='w':moving=True;direction=-1.;turn=0.;last_move=now
                    elif key in ('a','d'):moving=True;direction=-1.;turn=1. if key=='a' else -1.;last_turn=last_move=now
                    elif key=='s':moving=True;direction=1.;turn=0.;last_move=now
                    elif key=='x':moving=False;turn=0.
                    elif key=='e':moving=False;control(True)
                    elif key in ('g','r'):
                        moving=False;pub.publish(Twist());grip.publish(String(data='GRASP' if key=='g' else 'RELEASE'))
                    elif key in (' ','q','\x03'):
                        moving=False;pub.publish(Twist());control(False)
                        if key in ('q','\x03'):return
            now=time.monotonic()
            if now-last_publish>=.05:
                msg=Twist()
                if moving and now-last_move<=.25:
                    msg.linear.x=direction*.02
                    msg.angular.z=turn*.02
                pub.publish(msg);last_publish=now
    finally:
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old)
        if rclpy.ok():
            pub.publish(Twist());control(False);end=time.monotonic()+.2
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
