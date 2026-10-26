# 리더 인계: 수동 직선 협동운반

2026-10-27. 팔로워 작업공간에서 구현한 신규 패키지 `cooperative_manual`.
목표는 **한 터미널의 ↑/↓로 양쪽 직선 이동, G로 양쪽 파지, R로 양쪽 열기**다. 리더와 팔로워는 반대 차체 속도를 사용한다. 별도 경로 계획, 의도적인 추종 지연, 회전·리프트 동작은 포함하지 않는다.

이번에는 코드·빌드·격리 ROS 시험만 수행했다. 실제 두 로봇으로 짐을 잡은 주행은 아직 시험하지 않았다.

## 리더에서 패키지 내려받기

리더 기준 저장소: `rbgusrns/damgc_robot`, 확인한 main `c411ca2`.
리더는 COOPERATION 입력과 `enable_nav2_goal_selection` 파라미터가 있는 현재 selector를 사용한다.
팔로워 저장소의 오래된 leader selector를 리더에 배포하지 않는다.

리더 작업공간에서 신규 패키지와 이 문서만 가져온다.

```bash
cd ~/damgc_robot
# 리더 로컬 수정 상태 확인
git status --short
git fetch https://github.com/kmhuh1525/damgc_robot.git codex/cooperative-mission-20260930
git restore --source=FETCH_HEAD --worktree -- \
  src/cooperative_manual docs/MANUAL_STRAIGHT_TRANSPORT_HANDOFF.md
source /opt/ros/humble/setup.bash
colcon build --packages-select cooperative_manual --symlink-install
source install/setup.bash
```

위 명령의 대상 경로에 이미 별도 수정이 있으면 먼저 보관한다. main 전체를 바꾸거나 기존 매핑·Nav2 코드를 교체하는 과정은 필요 없다. 신규 패키지는 `nav2_msgs`, Dynamixel SDK를 import하지 않으며 호스트 ROS에서 실행할 수 있다.

## 사용하는 입력·출력

```text
전용 키보드 → /cooperation/manual/input
                    ↓
Leader manual_straight: 역방향 제한 + 공통 ramp + 양방향 상태/ACK 확인
    ├─ /leader/cooperation/cmd_vel → 기존 Leader selector COOPERATION → 기존 bridge
    └─ /cooperation/manual/leader (session/seq/linear)
                                ↓ DDS
Follower manual_straight: session/seq/freshness 확인, v_follower=-v_leader
    ├─ /follower/mission/cmd_vel → selector COOPERATION → 기존 guard → bridge
    └─ /cooperation/manual/follower (상태/heartbeat/명령 ACK) → Leader
```

↑는 리더 차체 전진·팔로워 후진, ↓는 리더 차체 후진·팔로워 전진이다. 두 로봇이 마주 보며 같은 월드 방향으로 움직이는 배치여야 한다. 각속도 출력은 항상0이다. 다른 자세·차체 방향의 자동 판정은 하지 않는다.

| 항목 | 값 |
|---|---|
| 명령/프로토콜 갱신 | 50Hz, 최신 샘플 queue depth1 |
| 리더 / 팔로워 속도 제한 | 각각±0.03m/s, 서로 반대 부호 |
| 공통 가속·감속 | 0.03m/s²; 정지→최대속도 약1초 |
| 수동 입력·상대 통신·명령 ACK 만료 | 0.25초 |
| odometry 수신 만료 / 원본 시각 한계 | 0.35초 / 0.50초 |
| enable 시 정지 판정 | 선속도 절댓값≤0.01m/s, 각속도≤0.03rad/s |
| arming/초기 연결 제한시간 | 8초. 준비된 팔로워가 아직 리더 세션 없이 대기하는 시간은 제외 |
| 리더 odom | `/leader/odometry/local` |
| 팔로워 odom | `/follower/odom/raw` |

자동 경로 peer의 PREPARE/READY 메시지와 섞이지 않게 `/cooperation/manual/*` 전용 통신을 사용한다. 제어 노드는 단조 시계로 수신·ACK 만료를 판단하며 두 장비의 동일한 odom 원점이나 동일한 절대시계를 요구하지 않는다.

## 실행 전 명령 주체 정리

- 로봇은 양쪽이 멈추고 마주 보는 직선 자세, 중립 힌지, 이미 잡은 상태에서 준비한다. G/R 기능을 사용할 때는 양쪽의 기존 Dynamixel 노드가 정상 연결돼 있어야 한다. 리프트 명령은 만들지 않는다.
- 기존 `cooperative_transport/transport_peer`, `leader_cooperation`, 협동 mission executor와 동시 운용하지 않는다. idle peer도 같은 출력 토픽의 publisher이면 enable이 거부된다.
- 기존 방향키 teleop 터미널과 자동 집기 시퀀스의 명령 발생을 멈춘다. 전용 키보드만 사용한다.
- 기존 리더 mapping/selector/bridge는 재사용한다. Nav2 주행과 일반 방향키 조작을 멈춘 상태에서 준비한다. enable 시 Nav2 자동 source 선택을 끈다.
- 기존 bridge·selector·guard가 있으면 중복 실행하지 않는다. 출력 토픽 publisher가 자기 노드 하나인지 검사한다.
- 팔로워 기존 자동 집기 프로세스를 유지한다면 DONE이고 approach/guard가 정지한 뒤 준비한다. 운반 중 `/follower/grasp/start`를 호출하지 않는다.

## 팔로워 실행

두 장비가 같은 ROS domain과 DDS transport를 사용한다. 현재 장비 구성의 예:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot_mission/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

**현재 집기용 selector·guard·bridge를 그대로 재사용할 때:** 이 selector의 COOPERATION 입력은 `/follower/cmd_vel`이므로 다음 인자를 사용한다.

```bash
ros2 launch cooperative_manual manual_straight.launch.py \
  role:=follower motion_enabled:=true command_topic:=/follower/cmd_vel \
  gripper_enabled:=true close_raw:=331
```

기존 selector가 `/follower/mission/cmd_vel`을 구독하는 transport용 구성이라면 `command_topic` 인자를 생략한다. **앞뒤 이동에는 기존 final guard를 `allow_reverse:=true`로 기동해야 한다.** 실행 중 ROS 파라미터만 바꿔서는 guard 내부 설정이 갱신되지 않으므로, 베이스 정지 상태에서 기존 guard만 재기동한다. bridge는 유지해 odom 원점을 보존한다. 새 노드는 reverse 설정이 false이면 arming을 거부한다.

**구동 노드가 없는 별도 팔로워 실행:** 기존 구동 launch를 종료한 상태에서만 실행한다.

```bash
ros2 launch cooperative_manual manual_straight.launch.py \
  role:=follower motion_enabled:=true drive_nodes:=true \
  use_stm32_bridge:=true i2c_write_enabled:=true \
  gripper_enabled:=true close_raw:=331
```

신규 launch의 motion/drive/bridge/I2C 쓰기/gripper 기능은 모두 기본 false다. 기본 실행만으로 drive bridge를 추가하거나 움직이지 않는다. 기존 guard의 선형 가속도 제한은 공통 ramp0.03m/s²보다 작지 않아야 한다. 현재 팔로워 guard0.05는 이 조건을 만족한다.

팔로워 준비 요청:

```bash
ros2 service call /follower/manual_straight/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /cooperation/manual/follower/status
```

`WAIT_PEER`가 되면 source와 guard는 준비됐지만 출력은0이고, 리더 세션을 기다린다. READY/armed는 물리 파지를 센서로 확인했다는 뜻이 아니다.

## 리더 실행·조작

리더 ROS 환경도 같은 domain/transport로 맞추고 기존 mapping selector/bridge·Dynamixel 노드가 실행된 상태에서 실행한다. `LEADER_CLOSE_RAW`에는 기존 리더 실물에서 확정한 집기값을 설정한다. 코드의 과거 기본값450/480을 실측 없이 선택하지 않는다. 팔로워의331을 리더에 복사하지 않는다.

리더 `open_raw` 기본1000, `gripper_settle` 기본25초다. 실제 속도·이동시간에 맞춰 바꿀 수 있고 팔로워 기본은9초다. G/R은 양쪽 ACK와 각각의 대기가 모두 끝나야 완료된다. 모터 속도 레지스터는 이 패키지가 바꾸지 않는다.

실행:

```bash
ros2 launch cooperative_manual manual_straight.launch.py \
  role:=leader motion_enabled:=true gripper_enabled:=true \
  close_raw:="${LEADER_CLOSE_RAW:?리더_확정_집기값을_설정하세요}"
```

별도 대화형 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 run cooperative_manual manual_straight_keys
```

| 키 | 동작 |
|---|---|
| `e` | 리더 enable 요청. 상대 준비·서비스 응답 뒤 `ACTIVE` 확인 |
| ↑ 또는 `w` 유지 | 리더 전진·팔로워 후진 |
| ↓ 또는 `s` 유지 | 리더 후진·팔로워 전진; 리더가 당기는 방향 |
| 키 해제 또는 `x` | 같은 ramp로 양쪽 감속 |
| `G` | 양쪽 베이스 정지 확인 후 양쪽 파지 |
| `R` | 양쪽 베이스 정지 확인 후 양쪽 그리퍼 열기 |
| Space | 베이스 정지 요청, gate 해제, 재활성화 요구 |
| `q`/Ctrl-C | 정지 요청 후 키보드 종료 |
| `a`/`d`/←/→ | 회전하지 않고 진행 입력 해제 |

일반 방향키 teleop 대신 이 전용 터미널만 사용한다. 키 자동반복 설정에 따라 처음 누를 때 짧은 입력 공백이 생길 수 있다. 입력 publisher가 끊기면 FAULT이고 새 publisher만으로 다시 움직이지 않는다. 방향을 바꿀 때도0속도를 거쳐 ramp로 전환한다.

G/R은 베이스를 먼저 감속하고 정지 odom을 확인한다. 이어 PREPARE→READY로 양쪽 설정과 모터 subscriber를 확인하고, COMMIT에서 양쪽 `[-1, target, -1, 1]` 명령을 실행한다. action ID로 재전송을 중복 실행하지 않는다. raw 대상은 각 노드가 자신의 보정값에서 선택한다. 그리퍼 동작 중 화살표 입력은 무시하며, 완료 뒤 입력을 한 번0으로 돌려야 다시 주행한다.

그리퍼 명령은 `/cooperation/manual/gripper/request`에 `GRASP` 또는 `RELEASE` String으로도 요청할 수 있다. 상태는 `/cooperation/manual/gripper/{leader,follower}/status`에서 본다. 모터 ACK3초 만료/ERROR 또는 준비 실패 시 베이스는 정지하며, 그리퍼는 마지막 목표를 유지한다. DONE은 모터 ACK와 이동 대기 완료이며 접촉 센서 판정이 아니다. 한 모터가 고장 났을 때 이미 실행된 반대 모터를 원자적으로 되돌릴 수는 없다. 명령 실행 시각에는 DDS·스케줄링·모터 지연이 존재한다.

외부 주행 입력은 `/cooperation/manual/input`의 Twist를 계속 갱신하면 된다. `linear.x`만 사용하고 나머지 축은0이어야 한다. 회전·측방·비유한 입력은 정지 사유다.

## 정지·재시작

```bash
ros2 service call /leader/manual_straight/enable std_srvs/srv/SetBool '{data: false}'
# 리더를 사용할 수 없을 때도 팔로워에서 독립 정지 가능
ros2 service call /follower/manual_straight/enable std_srvs/srv/SetBool '{data: false}'
```

일반 키 해제는 공통 감속이고, Space/disable/FAULT는 지연을 추가하지 않는 0속도·STOP 요청이다. 팔로워 guard 해제와 selector STOP도 요청한다. 통신이 완전히 끊기면 상대에게 STOP 패킷이 도달할 수 없으므로 각자의 watchdog이 정지시킨다. 물리적 동시 정지의 원자성이나0ms 출발 시차를 보장하지 않는다.

다시 출발하려면 원인을 확인하고 정지 odom 상태에서 팔로워 enable → 리더 `e`를 새로 수행한다. 통신 복구만으로 자동 재출발하지 않는다. 기존 Nav2 자동 선택 복구는 별도 작업이며, 수동 운반 종료 후 바로 자동 Nav2 주행으로 전환하지 않는다.

## 상태·오류 확인

```bash
ros2 topic echo /cooperation/manual/leader/status
ros2 topic echo /cooperation/manual/follower/status
ros2 topic echo /leader/cooperation/cmd_vel
ros2 topic echo /follower/mission/cmd_vel
ros2 topic info /leader/cooperation/cmd_vel -v
```

- `motion disabled`: 실제 시험 준비 후 launch에 `motion_enabled:=true`를 명시한다.
- `conflicting publisher`: 기존 peer/mission의 출력 소유권을 정리한다.
- `fresh stationary odometry required`: 실제 odom 토픽·시각·정지 여부를 확인한다.
- `selector/guard rejected arming`: 리더 최신 selector의 COOPERATION/자동 Nav2 선택 파라미터와 팔로워 guard를 확인한다.
- `manual input timeout`, `peer ... timeout`, `acknowledgement timeout`: 정지 후 입력 또는 DDS 연결을 확인하고 양쪽 재활성화한다.

## 무구동 검증

논리 테스트는 실제 ROS/motor 연결 없이 실행한다. ROS 통합 테스트는 아래의 **localhost 전용 별도 domain176**에서만 수행한다. 기본 domain의 pytest 실행에서는 통합 시험이 skip된다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 COOPERATIVE_MANUAL_ISOLATED_TEST=1 \
  python3 -m pytest -q src/cooperative_manual/test
```

시험에는 가짜 selector/guard 서비스와 합성 정지 odometry만 사용한다. 실제 bridge/I2C/그리퍼는 실행하지 않는다. 양방향 핸드셰이크, 공통 ramp와 부호, 일반 감속, operator stop, 통신·ACK·입력·odom timeout, 오래된 session/중복 seq 거부, 회전/측방 입력 거부, 앞뒤 부호 전환, G/R 정지 interlock·양쪽 ACK·중복 COMMIT 거부, 서비스 거부 정지를 확인한다.

검증 결과: 앞뒤 이동, 양쪽 그리퍼 트랜잭션, DDS/서비스 통합, 종료 처리를 포함해 총37개 통과. 신규 패키지 빌드 성공. 통합 시험은 localhost 전용 domain176에서 수행했으며 실제 장비 domain0에는 운반 명령을 보내지 않았다.

## 첫 실기 확인과 한계

먼저 짧은 직선으로 리더 후진·팔로워 전진 부호와 Space/통신 끊김 정지를 확인한다. 기존 그리퍼 목표는 유지하며 실제 출발·정지 시차, 바퀴 보정, 힘·미끄러짐은 추가 실기 확인 대상이다. 속도 프로파일을 공유하는 구조로서 오도메트리 상대 위치 폐루프 제어, 힘 제어, 장애물 감지·회피, 회전 운반은 구현하지 않았다. 실제 이동 속도는 바퀴·하중·통신·구동 지연에 따라 다를 수 있다.
