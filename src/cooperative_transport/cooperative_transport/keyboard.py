"""Extra B/N keyboard window for an already running mapping session."""
import os
import select
import sys
import termios
import time
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Open this controller in an interactive terminal')
    rclpy.init(args=args)
    node = Node('transport_keys')
    control = node.create_publisher(String, '/leader/mapping/control', 10)
    source = node.create_publisher(String, '/leader/command_selector/request', 10)
    node.create_subscription(String, '/leader/mapping/status', lambda msg: print('\r\n'+msg.data,flush=True), 10)
    print('COOP: B=prepare → NEW RViz goal → READY → N=start both. SPACE=stop both.\n'
          'Manual assembly only; no gripper keys or commands. Ctrl-C exits and stops cooperation.',flush=True)
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    last_key, last_time = None, 0.
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            if not select.select([fd],[],[],0)[0]:
                continue
            data = os.read(fd,32).decode('latin1')
            for key in data.lower():
                command = {'b':'COOP_PREPARE','n':'COOP_START',' ':'COOP_ABORT'}.get(key)
                if not command:
                    continue
                now = time.monotonic()
                if key == last_key and now-last_time < 1.:
                    last_time = now
                    continue
                last_key, last_time = key, now
                control.publish(String(data=command))
                if key == ' ':
                    source.publish(String(data='STOP'))
                print('\r\n'+command,flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,original)
        if rclpy.ok():
            control.publish(String(data='COOP_ABORT'))
            for _ in range(3):
                rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node()
        rclpy.try_shutdown()
