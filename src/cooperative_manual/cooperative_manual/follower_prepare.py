"""Probe/reuse or prepare the one follower stack; never sends a motion target."""
import argparse
from collections import Counter
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

REQUIRED=('command_selector','velocity_guard','stm32_bridge','dynamixel_orin_node','manual_straight')
CONFLICTS=('gripper_sequence','mission_executor','transport_peer')


def graph(node):
    return Counter(name for name,ns in node.get_node_names_and_namespaces() if ns=='/follower')


def parameters(node,name,keys):
    client=node.create_client(GetParameters,'/follower/'+name+'/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError(name+' parameter service unavailable')
        future=client.call_async(GetParameters.Request(names=keys))
        rclpy.spin_until_future_complete(node,future,timeout_sec=2)
        if not future.done() or future.result() is None:raise RuntimeError(name+' parameter timeout')
        return future.result().values
    finally:node.destroy_client(client)


def main(args=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['straight','curve'],default='straight')
    parser.add_argument('--probe',action='store_true',help='0=complete graph,10=empty,11=partial/conflicting')
    parser.add_argument('--check-only',action='store_true')
    options=parser.parse_args(args)
    peer='manual_curve' if options.mode=='curve' else 'manual_straight'
    required=REQUIRED[:-1]+(peer,)
    conflicts=CONFLICTS+(('manual_straight',) if options.mode=='curve' else ('manual_curve',))
    prefix='/cooperation/curve' if options.mode=='curve' else '/cooperation/manual'
    rclpy.init(args=[]);node=Node('follower_one_command_prepare')
    state={'text':''}
    node.create_subscription(String,prefix+'/follower/status',lambda m:state.update(text=m.data),10)
    exit_code=0
    try:
        end=time.monotonic()+(2. if options.probe else 25.)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            counts=graph(node)
            complete=all(counts[name]==1 for name in required)
            if not options.probe and complete:break
        counts=graph(node)
        if options.probe:
            if not any(counts[name] for name in required+conflicts):exit_code=10
            elif all(counts[name]==1 for name in required) and not any(counts[name] for name in conflicts):
                print('기존 서브 수동 이동 구성을 재사용합니다.',flush=True)
            else:
                print('서브의 일부/충돌 노드가 이미 있습니다. 기존 실행 터미널을 종료한 뒤 다시 실행하세요: '+str({k:v for k,v in counts.items() if k in required+conflicts}),file=sys.stderr)
                exit_code=11
        else:
            if not all(counts[name]==1 for name in required) or any(counts[name] for name in conflicts):
                raise RuntimeError('필수 노드 누락 또는 중복/자동 집기 노드 충돌: '+str(dict(counts)))
            reverse=parameters(node,'velocity_guard',['allow_reverse'])[0]
            if reverse.type!=1 or not reverse.bool_value:raise RuntimeError('기존 guard의 allow_reverse가 false입니다. guard 재기동이 필요합니다.')
            config=parameters(node,peer,['role','motion_enabled','gripper_enabled','close_raw','command_topic'])
            if config[0].string_value!='follower' or not config[1].bool_value:raise RuntimeError('수동 노드의 follower/motion_enabled 설정이 맞지 않습니다.')
            if not config[2].bool_value or config[3].integer_value!=331:raise RuntimeError('팔로워 그리퍼 설정은 enabled=true, close_raw=331이어야 합니다.')
            motor=parameters(node,'dynamixel_orin_node',['robot','rx28_speed','lift_enabled'])
            if motor[0].string_value!='follower' or motor[1].integer_value!=40 or motor[2].bool_value:raise RuntimeError('모터 설정을 확인하세요: follower, speed40, lift disabled 필요')
            subscribers=node.get_subscriber_names_and_types_by_node('command_selector','/follower')
            if config[4].string_value not in {topic for topic,_ in subscribers}:raise RuntimeError('수동 명령 topic과 selector 입력 remap이 다릅니다.')
            if options.check_only:
                print('서브 설정 확인 완료: 후진 허용, 그리퍼331/속도40, 리프트 제외. '+state['text'],flush=True)
            else:
                # Do not reset a running session when reusing this command.
                deadline=time.monotonic()+2
                while not state['text'] and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
                if state['text'].startswith('ARMING:'):
                    end=time.monotonic()+4
                    while state['text'].startswith('ARMING:') and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
                if state['text'].startswith(('ACTIVE:','WAIT_PEER:')):
                    print('서브 준비됨: '+state['text'],flush=True)
                else:
                    client=node.create_client(SetBool,'/follower/'+peer+'/enable')
                    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('서브 enable 서비스 없음')
                    future=client.call_async(SetBool.Request(data=True));rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                    if not future.done() or future.result() is None or not future.result().success:
                        raise RuntimeError('서브 활성화 거부: '+(future.result().message if future.done() and future.result() else 'timeout'))
                    state['text']=''
                    deadline=time.monotonic()+4
                    while not state['text'].startswith(('WAIT_PEER:','ACTIVE:')) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
                    if not state['text'].startswith(('WAIT_PEER:','ACTIVE:')):raise RuntimeError('서브 준비 실패: '+state['text'])
                    print('서브 준비 완료: '+state['text'],flush=True)
                print('리더 터미널에서 e 후 화살표를 사용하세요. G=양쪽 파지, R=양쪽 열기, Space=정지.',flush=True)
    except Exception as exc:
        print('서브 준비 실패: '+str(exc),file=sys.stderr,flush=True);exit_code=1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if exit_code:raise SystemExit(exit_code)
