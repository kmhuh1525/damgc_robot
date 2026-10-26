# 시연: 시작 신호 → 양쪽 파지 → 메인 경로 운반

개회식은 `opening_ceremony_terminal.sh`로 키보드 직진만 수행한다. 이 문서는 별도의 **시연 모드**다. 시연에서는 메인(리더) 시나리오가 시작 서비스를 한 번 호출하고, 양쪽 파지가 완료되면 메인이 계획한 경로의 주행 명령을 제공한다. 서브는 메인의 실제 odom 경로를 힌지 모델로 변환하여 추종한다. 리프트는 사용하지 않는다.

**시작 전 두 로봇과 상자가 파지할 위치에 있어야 한다.** 이 새 서비스는 AprilTag 접근/재배치까지 시작하지 않는다. 기존 `/leader/grasp_pair/start`가 호출하는 독립 AprilTag 접근 태스크와 동시에 실행하지 않는다.

## 실행

각 워크스페이스에서 `source /opt/ros/humble/setup.bash` 후 `colcon build --packages-select cooperative_manual --symlink-install`로 빌드한다. 리더 저장소 `rbgusrns/damgc_robot`, 작업 브랜치 `codex/manual-curve-20261027`; 서브 저장소 `kmhuh1525/damgc_robot`, 브랜치 `codex/cooperative-mission-20260930`.

기존 리더 수동 곡선/직진 노드와 키보드 태스크는 종료한다. 시연 리더 노드가 기존 `manual_curve` 리더 노드를 대체하므로 둘을 같이 실행하지 않는다. 서브는 같은 `manual_curve` 노드를 계속 사용한다. 양쪽 motor/odom/selector/guard/bridge는 기존 단일 구성을 유지한다.

서브:

```bash
~/damgc_robot_mission/scripts/run_follower_demo.sh
```

기존 완전한 서브 곡선 구성이 있으면 재사용한다. 개회식 직진 모드라면 먼저 그 협동 태스크를 종료하고 곡선 구성을 준비한다. 부분 구성이면 기존 곡선 준비 명령의 진단에 따라 실행을 정리해야 한다. 이 명령은 임의로 모터 노드를 종료하지 않는다.

리더(기존 구동·Dynamixel·`/leader/odom/raw` 실행 필요):

```bash
~/damgc_robot/scripts/run_leader_demo.sh
```

기본 입력은 `/leader/demo/cmd_vel`이다. 메인의 Nav2가 후진 운반 경로를 계획하고 `/nav2/cmd_vel`을 출력한다면:

```bash
DEMO_COMMAND_TOPIC=/nav2/cmd_vel ~/damgc_robot/scripts/run_leader_demo.sh
```

두 방식 중 하나만 실행한다. 스크립트가 켜져도 시작 신호 전에는 파지·운반하지 않는다. ROS domain 기본0, FastDDS UDPv4이며 양쪽이 같아야 한다.

## 메인 시나리오 연동 계약

| 항목 | ROS 인터페이스 |
|---|---|
| 시연 시작 신호 | `/leader/demo_transport/start`, `std_srvs/srv/Trigger` |
| 단계/완료/오류 | `/leader/demo_transport/status`, `std_msgs/msg/String` |
| 메인 주행 명령 | 기본 `/leader/demo/cmd_vel`, `geometry_msgs/msg/Twist`; 위 환경변수로 변경 가능 |
| 운반 종료 | `/leader/demo_transport/finish`, `std_srvs/srv/Trigger` |
| 취소·베이스 정지 | `/leader/demo_transport/cancel`, `std_srvs/srv/Trigger` |

메인의 순서:

1. status를 먼저 구독한다. Nav2 경로 주행을 아직 시작하지 않는다.
2. `start`를 한 번 호출한다. 응답 `success=true`는 시작 요청 접수이며 파지 완료가 아니다.
3. `ARM_FOLLOWER → WAIT_ACTIVE → OPENING → GRASPING → TRANSPORT_READY`를 기다린다. 기본적으로 양쪽 열림1000, 이후 리더477/서브331 파지, 양쪽 ACK와9초 대기를 확인한다. DONE 판정은 새 action/session의 현재 transaction을 사용한다. 물리적으로 실제 상자를 잡았는지 센싱하는 기능은 아니다.
4. `TRANSPORT_READY`를 받은 뒤 기존 Nav2 `FollowPath` 또는 메인의 경로 주행을 시작한다. 그 컨트롤러의 Twist를 선택한 입력으로 연속 발행한다. 이때부터 상태는 `TRANSPORTING`이다. 파지 전 입력은 저장/재생하지 않는다.
5. 메인 경로가 완료되면 먼저0 Twist를 보내 정지시킨 뒤 `finish`를 호출한다. 노드는 두 베이스를 정지하고 `DONE`이 되며 그리퍼는 유지한다.
6. 어느 단계든 `FAULT`면 메인의 Nav2 목표도 취소한다. 서비스 `cancel`로도 정지할 수 있다. 연결이 회복되어도 자동 재출발하지 않는다.

리더의 최신 `scripts/paired_grasp.py`는 `/leader/grasp_pair/start`로 **각자 AprilTag 접근**을 실행한다. 이 서비스는 그대로 남아 있으며 새 시연 서비스로 자동 변경되지 않는다. 시연 시나리오 호출부에서는 이를 위 `demo_transport/start` 및 `TRANSPORT_READY` 대기로 교체한다. 또한 기존 `scripts/solo_delivery.py`의 `/leader/teleop/cmd_vel` 출력/TELEOP selector 전환을 함께 실행하면 안 된다. 시연은 새 노드가 COOPERATION selector를 소유하고 Nav2의 출력만 받아 주행한다. 리더 작업자는 시나리오의 계획·FollowPath 부분을 유지하고 파지/주행 소유권 연결을 이 계약으로 반영한다.

이 모듈은 Nav2 경로 계획이나 목적지 선택을 새로 만들지 않는다. 메인이 기존 계획기를 사용하고, 서브는 **공유된 전체 지도 경로가 아니라 메인의 실제 이동 이력**을 추종한다. 임의 고정 시간 지연 값은 사용하지 않는다.

## 터미널에서 신호 시험

먼저 해당 로봇 워크스페이스의 `install/setup.bash`를 source하고 동일 DDS 환경을 사용한다.

```bash
ros2 topic echo /leader/demo_transport/status
```

다른 터미널/메인 시나리오에서:

```bash
ros2 service call /leader/demo_transport/start std_srvs/srv/Trigger '{}'
```

`TRANSPORT_READY`가 나오면 메인의 후진 경로 주행을 시작한다. 단순히 서비스 응답만 보고 출발하면 안 된다. 상태 구독은 시작 전에 연결한다.

운반 종료 또는 즉시 취소:

```bash
ros2 service call /leader/demo_transport/finish std_srvs/srv/Trigger '{}'
ros2 service call /leader/demo_transport/cancel std_srvs/srv/Trigger '{}'
```

위 두 명령은 목적에 맞는 하나만 사용한다. cancel/finish는 그리퍼를 자동으로 벌리지 않는다. 재시작은 상자·로봇을 초기 배치한 뒤 새 start를 사용하며, 이미 들고 있는 상자를 유지한 채 재개하는 서비스가 아니다. 새 start 기본 동작에는 열기가 포함된다.

## 경로와 통신 조건

- 리더가 앞에서 뒤를 보며 끌고 서브가 전진하는 배치. 메인 경로 컨트롤러는 리더 body 기준 `linear.x<=0`를 출력해야 한다.
- 제자리 회전 금지, 곡률 절댓값≤1/m(회전 반경≥1m). 직선의 반대 방향 복귀는 수동 곡선 모드에서만 제공하며 시연 자동 입력은 후진 당기기만 받는다.
- 목표 선속도≤0.02m/s로 곡률을 유지하며 축소한다. 경로 입력을 빠르게 주어도 이 상한을 넘지 않는다.
- 실측 경로·저속 잡음 억제·보정 완화·가속도 변화 제한은 기존 서브 곡선 코드와 같다. 힌지는 정면 기준±60도 모델이다. 큰 추종 오차만으로 정지하지 않고 감속 보정한다.
- `TRANSPORT_READY`에서 아직 입력이 없으면0으로 대기한다. 최초 입력을 받은 뒤 입력이0.25초 넘게 끊기면 FAULT로 양쪽 정지한다. 종료 때는 입력을 끊기만 하지 말고 finish/cancel을 사용한다.
- peer 통신·odom·trail·그리퍼 오류는 계속 정지 조건이다. 키보드 수동 입력으로 파지 게이트를 우회할 수 없다.

## 검사 범위

실제 DDS를 쓰는 격리 domain176 가상 로봇 테스트로 새 시작 신호, OPEN→GRASP 순서, 양쪽 ACK 전 주행 차단, 반복 시작 거절, 경로 수신 후 양쪽 이동, 정상 종료, 새 파지 cycle, 입력 유실 정지, ACK 누락 정지, arming 중 취소와 늦은 enable 차단, 잘못된 전진/회전 입력 정지를 확인한다. 실제 Nav2와 상자·두 로봇을 연결한 주행은 별도 실기 검증이 필요하다. 현재 실행 중인 로봇 모드는 코드 작성/빌드만으로 바뀌지 않는다.
