# 협동 미션 실행 가이드 (QR 인식 → 파지 → 반대편 파지 → 동시 리프트 → 1초 협동 직진)

## 1. 시나리오와 구현 위치

| 단계 | 동작 | 구현 |
|---|---|---|
| 1 | 리더가 물체의 AprilTag(QR 대용, tag36h11 ID 0, 5 cm)를 제자리 회전 탐색으로 찾고 정밀 정렬 | 리더 `mission_coordinator` + 기존 `rescue_robot_apriltag`/`leader_approach_control` |
| 2 | 리더 ALIGNED → RX-28 집게 닫기 | 리더 `mission_coordinator` → `/leader/dynamixel/command` |
| 3 | 팔로워가 (선택) odometry 기동으로 반대편 이동 → 반대편 면 태그 탐색 → 정렬 → 집게 닫기 | 팔로워 `mission_executor` + 기존 팔로워 AprilTag 파이프라인 |
| 4 | 두 로봇 RX-64 동시 리프트 (ACK 기반, 차이 = DDS 한 홉) | 양쪽 미션 노드 |
| 5 | 리더 공통 속도를 팔로워가 부호 반전해 추종, 같은 방향 1.0 s 직진 후 정지·유지 | 양쪽 미션 노드 + 기존 velocity guard |

**STM32 펌웨어(m4_firmware)는 수정하지 않았다.** 새 코드는 모두 Jetson 쪽
`src/cooperative_mission` 패키지와 `scripts/run_cooperative_mission.sh`이다. 기존 패키지 코드도
수정하지 않았고 launch에서 파라미터/remap만 바꿔 재사용한다.

## 2. 준비물 체크리스트

- 물체의 리더 쪽 면과 반대 면에 각각 **tag36h11 ID 0, 5 cm** 태그 부착
  (반대 면에 다른 ID를 쓰면 팔로워 `approach.yaml`의 `target_tag_id`와
  `follower_approach_control/config/approach_controller.yaml`의 `target_tag_id`를 같이 바꾼다)
- 두 Orin 모두 같은 커밋으로 빌드, 같은 `ROS_DOMAIN_ID`
- 각 로봇 U2D2(`/dev/ttyUSB0`)와 STM32 I2C(`/dev/i2c-7`, 0x42) 연결
- 팔로워는 기본값에서 **물체 반대편을 바라보도록** 배치한다(시작 시 태그가 안 보이면 제자리 회전 탐색).
  옆에서 출발해야 하면 `follower_mission.yaml`의 `reposition_segments`로 돌아가는 경로를 지정한다.
- 하드웨어 E-stop 준비, 첫 시험은 바퀴를 띄우거나 넓은 공간에서 저속으로

## 3. 빌드

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select cooperative_mission && colcon test-result --verbose
```

Windows에서 복사한 스크립트는 실행 권한이 없을 수 있으므로 `bash scripts/...`로 실행하거나
`chmod +x scripts/run_cooperative_mission.sh`를 한 번 실행한다.

## 4. 실행

### 4.1 모터 없이 통신·상태기계 점검

```bash
# Follower Orin
ROS_DOMAIN_ID=42 COOP_USE_STM32_BRIDGE=0 bash scripts/run_cooperative_mission.sh follower
# Leader Orin
ROS_DOMAIN_ID=42 COOP_USE_STM32_BRIDGE=0 COOP_PEER_IP=192.168.0.7 \
  bash scripts/run_cooperative_mission.sh leader
```

그리퍼까지 빼고 점검하려면 양쪽에 `MISSION_GRIPPER=0`을 추가한다.

### 4.2 실제 실행

```bash
# 1) Follower Orin (먼저)
ROS_DOMAIN_ID=42 COOP_PEER_IP=192.168.0.6 bash scripts/run_cooperative_mission.sh follower

# 2) Leader Orin
ROS_DOMAIN_ID=42 COOP_PEER_IP=192.168.0.7 bash scripts/run_cooperative_mission.sh leader
```

리더 스크립트는 팔로워 상태를 확인한 뒤 **Enter를 눌러야** `/mission/start`를 호출한다.
진행 상태가 출력되고, `DONE`(또는 `FAULT`) 후 다시 Enter를 누르면 두 로봇이 함께 물체를
내려놓고 집게를 연다(`/mission/release`).

주요 환경 변수:

| 변수 | 기본값 | 의미 |
|---|---:|---|
| `MISSION_DIRECTION` | `forward` | 리더 기준 운반 방향 (`forward`: 리더 전진·팔로워 후진, `backward`: 반대) |
| `MISSION_SPEED` | `0.05` | 순항 속도 m/s (≤ 0.10) |
| `MISSION_DURATION` | `1.0` | 이동 시간 s (가감속 포함) |
| `MISSION_GRIPPER`, `MISSION_GRIPPER_PORT` | `1`, `/dev/ttyUSB0` | Dynamixel 사용 여부/포트 |
| `COOP_USE_STM32_BRIDGE`, `COOP_I2C_*` | 기존과 동일 | STM32 bridge 설정 |

### 4.3 수동 명령 (스크립트 없이)

```bash
# Follower
ros2 launch cooperative_mission follower_mission.launch.py
# Leader
ros2 launch cooperative_mission leader_mission.launch.py transport_direction:=forward
# 별도 터미널 (Leader)
ros2 service call /mission/start   std_srvs/srv/Trigger
ros2 service call /mission/abort   std_srvs/srv/Trigger
ros2 service call /mission/release std_srvs/srv/Trigger
ros2 service call /mission/reset   std_srvs/srv/Trigger
```

모니터링:

```bash
ros2 topic echo /mission/state                      # 리더 미션 상태
ros2 topic echo /leader/mission/status              # 상세 사유 + 팔로워 상태(JSON)
ros2 topic echo /follower/mission/status            # 팔로워 상태/ACK(JSON)
ros2 topic echo /cooperation/target_velocity        # 공통 속도(리더 좌표)
ros2 topic echo /leader/cmd_vel ; ros2 topic echo /follower/safe_cmd_vel
```

**동시에 실행하면 안 되는 것**: `leader_cooperation`, `leader_apriltag_drive.launch.py`,
`follower_apriltag_drive.launch.py`, `follower_cooperation_drive.launch.py`,
`gripper_sequence.launch.py`, 방향키 teleop. 같은 명령 토픽을 발행해 충돌한다
(리더는 `/mission/start` 시점에 중복 publisher를 검사해 거부한다).

## 5. 상태 흐름

```text
Leader : IDLE → LEADER_SEARCH ⇄ LEADER_APPROACH → LEADER_GRASP → FOLLOWER_APPROACH
         → LIFT_PREPARE → LIFT → TRANSPORT_PREPARE → TRANSPORT → TRANSPORT_FINISH → DONE
         (DONE/FAULT) --/mission/release--> RELEASE → IDLE
Follower: IDLE → [REPOSITION] → SEARCH ⇄ APPROACH → GRASP → GRASPED → LIFT_READY
         → LIFTING → LIFTED → TRANSPORT_READY → HOLD --RELEASE--> RELEASING → IDLE
어느 단계든 오류/timeout/abort → FAULT
(속도 0, 양쪽 guard off, 팔로워 selector STOP, 그리퍼 유지)
```

## 6. 현장 보정 항목

실기에서 반드시 확인·조정할 값이다(모두 YAML/launch 인자로 조정 가능).

1. 그리퍼 raw 값: 리더 open/close `1000/450`, 팔로워 `950/350`, RX-64 lift/lower `300/600`
   (기존 launch 값에서 가져옴). 물체 높이에 맞게 `lift_raw`를 조정한다.
2. `lift_duration`(3.5 s): RX-64 속도 50에서 실제 도달 시간 + 여유.
3. 파지 거리: 리더 `final_target_distance`(0.23 m), 팔로워 `base_target_forward`(0.25 m).
   두 로봇 파지 후 물체를 사이에 두고 벌어지거나 밀리지 않는지 확인.
4. 운반 방향 부호: 첫 시험은 `MISSION_SPEED=0.03`으로 두 로봇이 같은 월드 방향으로 움직이는지
   확인한다(팔로워가 반대로 움직이면 즉시 abort 후 모터 배선/부호 점검).
5. 탐색 방향 `search_direction`(+1 CCW / -1 CW)과 `search_max_angle_deg`.

## 7. 문제 해결

| 증상 | 확인 |
|---|---|
| start 거부 `Follower mission status not received` | DDS domain/네트워크, 팔로워 먼저 실행 |
| start 거부 `Follower is HOLD/FAULT, expected IDLE` | `/mission/reset` 또는 `/mission/release` |
| start 거부 `not ready: ... unavailable` | 해당 노드가 떠 있는지 `ros2 service list` |
| start 거부 `another node publishes /cooperation/target_velocity` | `leader_cooperation` 종료 |
| `FAULT: no confirmation for ...` | 서비스 응답 없음(노드 멈춤) |
| `FAULT: Leader gripper error` | U2D2 포트/전원, `/leader/dynamixel/status` |
| `FAULT: object tag not found` | 태그 조명/크기/ID, 탐색 각도 |
| `FAULT: Follower status timeout` | 무선 품질, 팔로워 노드 상태 |

스크립트/launch를 종료하면 Dynamixel 노드가 토크를 끄므로 **들어 올린 상태에서 종료하면
물체가 떨어진다.** 종료 전 `/mission/release`로 내려놓는다.
