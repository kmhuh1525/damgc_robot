"""Read-only startup routing for the ceremony; hardware owners stay singular."""
import time
import rclpy
from rclpy.node import Node
from .follower_prepare import graph,REQUIRED,CONFLICTS


def classify(counts):
    if any(counts[n] for n in CONFLICTS+('manual_curve',)) or any(counts[n]>1 for n in REQUIRED):
        return 11
    if all(counts[n]==1 for n in REQUIRED):return 0
    if not any(counts[n] for n in REQUIRED):return 10
    if all(counts[n]==1 for n in REQUIRED[:-1]) and not counts['manual_straight']:return 12
    return 11


def main():
    rclpy.init();node=Node('opening_ceremony_probe')
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        counts=graph(node);result=classify(counts)
        if result==11:
            print('개회식 시작 불가: 곡선/다른 태스크 실행을 먼저 종료하고 재실행하세요. '+str(dict(counts)),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(result)
