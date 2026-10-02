# Leader FINAL_APPROACH Tag-Loss Grace and Gripper Integration

> **Current integrated-launch behavior (2026 update)**
>
> The active Leader integration is now provided by
> `rescue_robot_bringup/launch/leader_apriltag_drive.launch.py`. Its defaults are
> `gripper_enabled=true`, RX-28 `open_raw=1000`, RX-28 `close_raw=450`, and
> `lift_enabled=true`, `lift_raw=300`, and RX-64 Moving Speed raw `50`. The active sequence is
> `/leader/supply/detected=true` -> RX-28 OPEN 1000 -> existing approach ->
> visual alignment satisfied -> mandatory post-align odometry advance ->
> final `/leader/base_alignment/state=ALIGNED` -> RX-28 CLOSE 450 -> existing
> `close_wait=3.0 s` -> RX-64 Goal Position 300 -> DONE. The old
> `rx64_middle` automatic step described in historical sections below is no longer
> part of the integrated sequence; the semantic command remains available for
> manual Dynamixel tests. Automatic lift can be disabled without source changes
> with `lift_enabled:=false`.
>
> `gripper_enabled=false` is the top-level master gate and excludes both the
> Dynamixel node and sequence launch. It does not change the existing wheel safety:
> `velocity_guard` remains startup-disabled. The integrated launch enables
> `post_align_odom_enabled=true` by default; `post_align_odom_enabled:=false`
> restores the former ALIGNED timing. The historical grace-only tests below
> apply with post-align disabled.

## Follower 적용값

이 문서의 Leader 시퀀스를 Follower에 적용할 때는 Leader의 거리·모터 ID·raw
범위를 그대로 사용하지 않는다. 현재 Follower 소스 기준 값은 다음과 같다.

| 항목 | Follower 값 | 설명 |
|---|---:|---|
| camera `target_distance` | `0.15 m` | 기존 Follower AprilTag 접근 기준 |
| `base_target_forward` | `0.25 m` | Follower base-frame 최종 목표 |
| `base_forward_tolerance` | `0.04 m` | 최종 전방 허용 오차 |
| `base_lateral_tolerance` | `0.04 m` | 최종 횡방향 허용 오차 |
| `base_bearing_tolerance_deg` | `8.0 deg` | 최종 yaw 허용 오차 |
| `base_stable_time` | `0.30 s` | ALIGNED 안정 시간 |
| `aligned_confirm_samples` | `3` | ALIGNED fresh confirmation 횟수 |
| `stabilizing_tag_loss_grace_sec` | `0.30 s` | STABILIZING tag-loss grace |
| `final_approach_tag_loss_grace_sec` | `0.30 s` | FINAL_APPROACH tag-loss grace |
| RX-28 | ID `1`, raw `1..1021` | Follower gripper |
| RX-64 | ID `50`, raw `260..670` | Follower lift |
| OPEN | RX-28 raw `950` | Follower 기본 OPEN |
| CLOSE | RX-28 raw `350` | Follower 기본 CLOSE |
| LIFT | RX-64 raw `300` | Follower 설정값 |

Follower의 authoritative alignment topic은
`/follower/base_alignment/state`이며 detection topic은
`/follower/supply/detected`이다. 따라서 Follower 시퀀스는 다음 순서로
동작한다.

```text
/follower/supply/detected=true
  -> RX-28 OPEN 950
  -> /follower/base_alignment/state=ALIGNED
  -> RX-28 CLOSE 350
  -> close_wait(3.0 s)
  -> RX-64 LIFT 300
  -> DONE
```

Follower launch에서는 Follower의 검증된 설정값으로 `lift_enabled`를 `true`로
두어 raw 300 lift까지 자동 실행한다. lift를 제외한 시험이 필요하면 다음처럼
명시적으로 비활성화한다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  lift_enabled:=false
```

Follower 관련 명령 토픽은 `/follower/dynamixel/command`, 상태 확인 토픽은
`/follower/dynamixel/status`와 `/follower/sequence/status`이다. 이 Follower 적용값은
Leader의 `final_target_distance=0.23 m`, RX-28 ID 2, RX-64 ID 33,
`open_raw=1000`, `close_raw=450` 기준과 다르다.

## Current launch arguments and verification

| Argument | Default | Safety meaning |
|---|---:|---|
| `gripper_enabled` | `true` | Includes/excludes the complete Dynamixel subsystem |
| `gripper_open_raw` | `1000` | RX-28 OPEN goal |
| `gripper_close_raw` | `450` | RX-28 CLOSE goal |
| `lift_enabled` | `true` | Switch for automatic RX-64 lift |
| `lift_raw` | `300` | RX-64 Goal Position; accepted Leader range is 260..670 |
| `rx64_speed` | `50` | RX-64 Moving Speed raw value |

Start the complete integration with wheel output still safely blocked:

```bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

Inspect `/leader/supply/detected`, `/leader/base_alignment/state`,
`/leader/dynamixel/command`, `/leader/dynamixel/status`, and `/sequence/status`.
The expected default cycle is OPEN 1000, ALIGNED, CLOSE 450, wait 3.0 seconds,
RX-64 300, and DONE. Enable wheels only after the existing safety checks:

```bash
ros2 service call /leader/velocity_guard/enable std_srvs/srv/SetBool "{data: true}"
```

### Dynamixel ROS write-path diagnosis

The Leader RX-28 uses Dynamixel Protocol 1.0, ID `2`, baudrate `115200`,
Torque Enable address `24`, and Goal Position address `30`. The ROS node uses the
ROBOTIS Python SDK (`PortHandler` and `PacketHandler(1.0)`) for serial direction,
packet construction, and response validation. The ROS raw interfaces remain:

```text
[rx64_raw, rx28_raw, torque]
[rx64_raw, rx28_raw, rx64_torque, rx28_torque]
```

`-1` leaves that position or torque field unchanged. For example,
`[-1, 1000, -1, 1]` enables only RX-28 torque and writes RX-28 Goal Position
`1000`; RX-64 is not touched. `OK` status is emitted only after the SDK reports
`COMM_SUCCESS` and packet error `0`. Failures identify the operation, such as
`ERROR RX28 position=1000: [TxRxResult] ...`.

RX-64 Moving Speed uses address `32`, a 2-byte SDK write, and default raw value
`50` for both robot profiles. It is configured once after the bus opens and before
any RX-64 Goal Position. This register write does not enable torque or command
motion. Raw `0` means unrestricted maximum speed, not stop. RX-64 raw `500 → 300`
at speed `50` has been hardware-validated. RX-28 Moving Speed is not written, so
its existing OPEN/CLOSE speed behavior remains independent and unchanged.

Troubleshoot in this order:

1. Confirm `/dev/ttyUSB0`, baudrate `115200`, Protocol 1.0, and RX-28 ID `2`.
2. Run the SDK ping test.
3. Run the SDK direct test: `write1ByteTxRx(ID=2, address=24, data=1)` followed by
   `write2ByteTxRx(ID=2, address=30, data=<target>)`.
4. If SDK ping/direct motion succeeds but ROS does not, inspect
   `/leader/dynamixel/status` for the SDK TxRx or packet-error message and treat it
   as a software driver-path failure.

The package declares the `dynamixel_sdk` ROS runtime dependency; install the ROS
Humble package on a deployed machine with `ros-humble-dynamixel-sdk` when it is
not already present.

For AprilTag-only regression, restart with:

```bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py \
  gripper_enabled:=false
```

`ros2 node list` must then contain no `dynamixel_orin_node` or `gripper_sequence`;
the camera, alignment, approach, guard, and STM32 nodes remain present. To retain
RX-28 OPEN/CLOSE while disabling only automatic lift, use `lift_enabled:=false`.
An unset, negative,
non-finite, or out-of-range value produces a warning/error and skips RX-64 motion;
it never crashes the node or undoes the RX-28 CLOSE.

## 1. 목적과 변경 배경

이 변경 전에도 Leader의 COARSE, NEAR, FINAL_YAW_ALIGN, FINAL_APPROACH,
STABILIZING 및 ALIGNED 판정은 실제 로봇에서 검증되어 있었다. 검증된 기준은
`final_target_distance=0.23 m`, `final_position_tolerance=0.020 m`,
`final_yaw_tolerance_deg=5.0 deg`, `base_stable_time=0.30 s`, fresh sample
confirmation 3회, STABILIZING tag-loss grace 0.20초 및 ALIGNED latch다.

이번 변경은 다음 두 문제만 해결한다.

1. FINAL_APPROACH에서 순간적인 visual dropout이 즉시 TAG_LOST를 만들었다.
2. gripper sequence가 검증된 `/leader/base_alignment/state`가 아니라 기존
   camera-frame `/leader/alignment/state`를 기본 입력으로 사용했다.

변경 전 흐름은 다음과 같았다.

```text
FINAL_APPROACH
|
+-- tag visible ------> existing visual control
|
+-- temporary loss ---> TAG_LOST ---> STOP/reset

tag detected -> RX-28 OPEN
             -> /leader/alignment/state
             -> ALIGNED -> RX-28 CLOSE -> RX-64 LIFT
```

## 2. 변경 후 FINAL_APPROACH 동작

```text
FINAL_APPROACH
|
v
fresh AprilTag observation lost
|
v
zero command, no target pose
|
v
visual grace: at most 0.20 s from the last fresh observation receipt
|
+-- fresh observation returns within grace
|      |
|      v
|   resume the existing FINAL_APPROACH control with the new pose
|      |
|      v
|   STABILIZING -> confirmation x3 -> ALIGNED latch
|
+-- elapsed time > 0.20 s
       |
       v
    TAG_LOST -> existing STOP/reset
```

Grace는 이동 기능이 아니다. Grace 중 `/leader/supply/detected=false`, selected tag
ID는 `-1`이고, target pose가 없는 `LeaderAlignmentCommand`가 발행된다. 이 command의
control 값은 zero이며 controller도 detection 및 pose validity gate에서 zero를 만든다.
마지막 pose, cached TF, stale target 및 odometry는 control에 전달되지 않는다.

## 3. Fresh observation과 두 grace의 구분

`_accept_observation_stamp()`는 tag ID별 source timestamp가 이전 값보다 엄격히 클 때만
`is_new_observation=True`를 반환한다. Duplicate/cached TF는 fresh observation이 아니므로
FINAL_APPROACH grace 기준 시각을 갱신하지 않는다.

Fresh base decision이 정확히 `state=FINAL_APPROACH` 및
`mode=FINAL_APPROACH`일 때 `_record_final_approach_observation()`이 node의 현재 시각을
`_last_fresh_final_observation_time`에 저장한다. Dropout에서는 다음 값을 직접 계산한다.

```text
dropout_age = current_node_time - last_fresh_final_observation_receipt_time
```

`dropout_age <= final_approach_tag_loss_grace_sec`인 동안 zero를 유지한다.
`dropout_age > final_approach_tag_loss_grace_sec`이면 즉시 기존 lost 처리를 실행하며,
이 경로에서는 blind handoff도 명시적으로 허용하지 않는다.

두 grace는 서로 다른 계층과 의미를 가진다.

- FINAL_APPROACH grace: visual control을 중지하고 fresh tag를 최대 0.20초 기다린다.
- STABILIZING grace: 이미 최종 tolerance 안에서 안정성을 확인하던 상태의 짧은 loss를
  기존 `BaseAlignmentStateMachine`이 처리한다.

`blind_last_tag_max_age=0.25 s`는 기존 blind snapshot/odometry freshness를 위한 값이다.
FINAL_APPROACH grace는 이 값 이후 시작되지 않으며 두 시간이 더해지지 않는다. Duplicate
TF도 마지막 fresh receipt부터 0.20초가 지나면 TAG_LOST가 된다.

이 절의 과거 grace-only 시험에서는 `blind_final_approach_enabled=false`를
사용했다. 현재 작업 YAML의 blind 설정은 true이며 post-align은 별도 기능이다.
당시 grace-only 구현은 다음을 사용하지 않았다.

- odometry
- `BLIND_FINAL_APPROACH`
- `compute_blind_remaining_distance()`
- 마지막 AprilTag pose 기반 전진

## 4. 관련 parameter

| Parameter | 값 | 단위 | 역할 | 이번 변경 |
|---|---:|---|---|---|
| `final_target_distance` | 0.23 | m | tag plane과 최종 base_link 사이 거리 | 변경 없음 |
| `final_position_tolerance` | 0.020 | m | 최종 위치 허용 오차 | 변경 없음 |
| `final_yaw_tolerance_deg` | 5.0 | deg | 최종 yaw 허용 오차 | 변경 없음 |
| `base_stable_time` | 0.30 | s | STABILIZING 최소 안정 시간 | 변경 없음 |
| `aligned_confirm_samples` | 3 | samples | 안정 시간 이후 필요한 fresh confirmation | 변경 없음 |
| `stabilizing_tag_loss_grace_sec` | 0.20 | s | 기존 STABILIZING dropout grace | 변경 없음 |
| `final_approach_tag_loss_grace_sec` | 0.20 | s | 마지막 fresh FINAL_APPROACH observation 이후 stop-and-wait 시간 | 추가 |
| `blind_final_approach_enabled` | false (당시 시험값) | bool | blind odometry 접근 허용 여부 | 현재 YAML 값과 구분 |
| `blind_last_tag_max_age` | 0.25 | s | 기존 blind용 마지막 visual/odom freshness | 변경 없음 |

이 표는 당시 시험 기록이다. 현재 값은 `rescue_robot_apriltag/config/approach.yaml`과
node startup parameter에서 확인한다. FINAL_APPROACH grace는 startup-only이며
음수 또는 비유한 값은 거부된다.

## 5. 기존 정렬 상태머신 보존

`base_alignment_logic.py`는 수정하지 않았다. 다음 검증 경로도 그대로다.

```text
STABILIZING
  -> existing 0.20 s stabilizing loss grace
  -> 0.30 s stability
  -> 3 fresh confirmations
  -> ALIGNED
  -> ALIGNED latch, including subsequent tag loss
```

COARSE, NEAR, FINAL_YAW_ALIGN과 정상적인 visual FINAL_APPROACH control 계산도 변경하지
않았다. FINAL_APPROACH grace 중에는 `BaseAlignmentStateMachine.update(None, ...)`를
호출하지 않으므로 short dropout이 기존 phase latch를 reset하지 않는다. Grace timeout
이후에만 기존 TAG_LOST/reset 경로를 호출한다.

## 6. Historical standalone gripper behavior

The remainder of this section records the previously validated standalone
sequence for historical reference. It is superseded for the integrated launch by
the current behavior at the top of this document: OPEN 1000, CLOSE 450, and
automatic RX-64 raw 300 lift at Moving Speed raw 50.

아래 표는 당시 standalone profile을 기록한 것이며 현재 integrated profile
`260..670`의 설정값이 아니다.

| Actuator | 용도 | ID | min | max | sequence command |
|---|---|---:|---:|---:|---|
| RX-28 | gripper open/close | 2 | 1 | 1021 | historical open `1021`, close `450` |
| RX-64 | lift | 33 | 450 | 775 | `rx64_middle` = raw `612` |

The old standalone sequence used `open_raw=1021`, `close_raw=450`,
`close_wait=2.0 s`, then emitted `rx64_middle`. This behavior is retained here
only as historical context; the integrated sequence no longer emits that lift
command automatically.

Authoritative topic 연결은 다음과 같이 바뀌었다. 두 topic의 message type은 모두
`std_msgs/msg/String`이다.

```text
Before: gripper_sequence -> /leader/alignment/state
After:  gripper_sequence -> /leader/base_alignment/state
```

`/leader/base_alignment/state`는 실제 Leader 이동 제어에서 사용하고 검증한 base-frame
상태이므로 CLOSE는 이 topic의 값이 정확히 `ALIGNED`일 때만 실행된다.

최종 흐름은 다음과 같다.

```text
/leader/supply/detected=true
  -> RX-28 OPEN (raw 1021)
  -> existing Leader approach
  -> /leader/base_alignment/state=ALIGNED
  -> RX-28 CLOSE (raw 450)
  -> wait 2.0 s
  -> RX-64 rx64_middle (raw 612)
  -> wait 2.0 s
  -> DONE
```

### Gripper state machine

| 상태 | 진입/유지 조건 | command | 다음 상태 |
|---|---|---|---|
| `WAITING_FOR_TAG` | startup 또는 다음 cycle arm | 없음 | detection=true이면 `OPENING` |
| `OPENING` | 첫 detection | RX-28 raw 1021 한 번 | alignment=`ALIGNED`이면 `CLOSING` |
| `CLOSING` | base ALIGNED | RX-28 raw 450 한 번 | 2.0초 후 `LIFTING` |
| `LIFTING` | close wait 완료 | `rx64_middle` 한 번 | 추가 2.0초 후 `DONE` |
| `DONE` | sequence 완료 | 없음 | detection=false이고 alignment!=`ALIGNED`일 때만 re-arm |

Repeated detection은 OPENING에서 무시되고 repeated ALIGNED는 CLOSING/LIFTING/DONE에서
새 close를 만들지 않는다. DONE에서 detection이 잠깐 false가 되어도 ALIGNED latch가
유지되는 동안에는 재무장하지 않으므로 같은 cycle이 다시 실행되지 않는다.

## 7. 수정 파일

| File | 변경 |
|---|---|
| `rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py` | fresh receipt 기반 FINAL_APPROACH stop-and-wait grace |
| `rescue_robot_apriltag/config/approach.yaml` | 새 0.20초 parameter |
| `rescue_robot_apriltag/test/test_base_pose.py` | grace, duplicate, timeout, zero 및 non-final regression 테스트 |
| `rescue_robot_tools/scripts/gripper_sequence_node.py` | authoritative topic과 DONE re-arm 조건 |
| `rescue_robot_tools/launch/gripper_sequence.launch.py` | 동일한 alignment topic 기본값 |
| `rescue_robot_tools/test/test_gripper_sequence_node.py` | OPEN/CLOSE/LIFT 및 중복 방지 테스트 |
| `rescue_robot_tools/CMakeLists.txt`, `package.xml` | gripper pytest 등록 |
| 이 문서 | Jetson 검증 및 운영 절차 |

의도적으로 변경하지 않은 항목은 COARSE, NEAR, FINAL_YAW_ALIGN, 정상 FINAL_APPROACH
controller, STABILIZING 0.30초, confirmation 3회, 기존 STABILIZING grace 0.20초,
ALIGNED latch, `final_target_distance=0.23`, `final_position_tolerance=0.020`,
`final_yaw_tolerance_deg=5.0`, velocity guard, STM32 I2C bridge, TF, RealSense/AprilTag
pipeline, follower 및 Dynamixel ID/min/max다. `leader_apriltag_drive.launch.py`도 변경하지
않았다.

## 8. Jetson build와 정적 테스트

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_apriltag rescue_robot_tools leader_approach_control
source install/setup.bash
colcon test --packages-select \
  rescue_robot_apriltag rescue_robot_tools leader_approach_control \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 9. Phase A: Gripper OFF, FINAL_APPROACH grace 검증

Gripper와 U2D2 node를 실행하지 않은 상태에서 먼저 주행만 검증한다. Launch startup에서
approach controller는 enabled, velocity guard는 disabled다.

Terminal 1:

```bash
cd ~/damgc_robot
source install/setup.bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

Terminal 2:

```bash
source ~/damgc_robot/install/setup.bash
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String
```

Terminal 3 — 주변 안전 확인 후에만 실제 motor output을 enable한다:

```bash
source ~/damgc_robot/install/setup.bash
ros2 service call /leader/approach/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /leader/velocity_guard/enable std_srvs/srv/SetBool "{data: true}"
```

Terminal 4:

```bash
source ~/damgc_robot/install/setup.bash
ros2 topic echo /leader/approach/cmd_vel_raw geometry_msgs/msg/Twist
# 별도 terminal에서 최종 guarded output 확인
ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist
```

시험 순서와 기대 결과:

1. Tag를 계속 보이면 `COARSE -> NEAR -> FINAL_YAW_ALIGN -> FINAL_APPROACH ->
   STABILIZING -> ALIGNED`가 된다.
2. FINAL_APPROACH 중 0.20초보다 짧게 가리면 즉시 zero가 되고 FINAL_APPROACH를 유지한다.
   Fresh tag가 돌아오면 새 pose로 FINAL_APPROACH를 재개한다.
3. FINAL_APPROACH 중 마지막 fresh observation 이후 0.20초를 초과해 가리면 TAG_LOST와
   zero가 된다.
4. ALIGNED 이후 tag를 가리면 기존 latch에 의해 ALIGNED와 zero가 유지된다.
5. 시험 중 tag가 보이지 않는데 `/leader/cmd_vel`이 non-zero이면 즉시 guard를 disable하고
   시험을 중단한다.

```bash
ros2 service call /leader/velocity_guard/enable std_srvs/srv/SetBool "{data: false}"
```

## 10. Phase B: Gripper 통합 검증

Phase A가 모두 정상일 때만 U2D2와 gripper sequence를 실행한다. 사람과 물체를 actuator
가동 범위 밖에 두고 `/dev/ttyUSB0`가 실제 U2D2인지 먼저 확인한다.

ROS보다 먼저 RX-28 단독 통신을 확인하려면 다음 SDK smoke test를 사용한다. 이 명령은
실제 RX-28을 움직이므로 actuator 주변을 비운 뒤 사용자가 직접 실행한다.

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(1.0)
if not port.openPort() or not port.setBaudRate(115200):
    raise RuntimeError('cannot open /dev/ttyUSB0 at 115200')
try:
    model, comm, error = packet.ping(port, 2)
    print('ping', model, comm, error)
    if comm != COMM_SUCCESS or error != 0:
        raise RuntimeError('RX-28 ping failed')
    comm, error = packet.write1ByteTxRx(port, 2, 24, 1)
    print('torque', comm, error)
    if comm != COMM_SUCCESS or error != 0:
        raise RuntimeError('RX-28 torque write failed')
    comm, error = packet.write2ByteTxRx(port, 2, 30, 1000)
    print('goal_position', comm, error)
    if comm != COMM_SUCCESS or error != 0:
        raise RuntimeError('RX-28 goal-position write failed')
finally:
    port.closePort()
PY
```

성공한 SDK ping/direct test 뒤 ROS 경로가 실패하면 `/leader/dynamixel/status`의
`ERROR RX28 ...` 내용을 기준으로 software driver path를 진단한다.

Terminal 1은 Phase A의 Leader drive를 그대로 유지한다.

Terminal 2:

```bash
source ~/damgc_robot/install/setup.bash
ros2 launch rescue_robot_tools dynamixel_orin.launch.py \
  robot:=leader port:=/dev/ttyUSB0 baudrate:=115200 \
  startup_pose_enabled:=false startup_torque:=false
```

Terminal 3:

```bash
source ~/damgc_robot/install/setup.bash
ros2 launch rescue_robot_tools gripper_sequence.launch.py enabled:=true
```

Terminal 4:

```bash
source ~/damgc_robot/install/setup.bash
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String
# 별도 terminal에서 각각 확인
ros2 topic echo /leader/dynamixel/status std_msgs/msg/String
ros2 topic echo /sequence/status std_msgs/msg/String
```

기대 동작은 detection 시 RX-28 OPEN, 기존 approach 완료 후 base ALIGNED에서만 RX-28
CLOSE, 2초 후 RX-64 LIFT, 추가 2초 후 DONE이다. DONE/ALIGNED 상태에서 tag를 잠깐
가렸다 다시 보여도 OPEN/CLOSE/LIFT가 다시 실행되면 안 된다.

## 11. ROS 2 확인 명령과 정상 로그

```bash
ros2 node list
ros2 topic list
ros2 topic echo /leader/supply/detected std_msgs/msg/Bool
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String
ros2 topic echo /leader/alignment/command leader_alignment_msgs/msg/LeaderAlignmentCommand
ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist
ros2 topic echo /leader/dynamixel/command std_msgs/msg/Float64MultiArray
ros2 topic echo /leader/gripper/command std_msgs/msg/String
ros2 topic echo /leader/dynamixel/status std_msgs/msg/String
ros2 topic echo /sequence/status std_msgs/msg/String
ros2 param get /leader/apriltag_approach final_approach_tag_loss_grace_sec
ros2 param get /leader/apriltag_approach blind_final_approach_enabled
ros2 param get /gripper_sequence alignment_topic
```

상태 변화 시 실제 코드에서 출력하는 주요 로그/상태는 다음과 같다.

```text
FINAL_APPROACH: tag temporarily lost; holding zero velocity
FINAL_APPROACH: fresh tag reacquired; resuming visual control
FINAL_APPROACH: tag loss grace expired; entering TAG_LOST
Base alignment state changed to STABILIZING
Base alignment state changed to ALIGNED
OPENING tag_detected
CLOSING alignment=ALIGNED close_raw=450
LIFTING
DONE
```

FINAL_APPROACH grace 로그는 상태 변화 때만 출력되며 timer frame마다 반복하지 않는다.

## 12. Troubleshooting

### FINAL_APPROACH에서 바로 TAG_LOST

`final_approach_tag_loss_grace_sec`, `/leader/apriltag_approach`의 실제 parameter,
source timestamp가 증가하는지, 마지막 fresh observation이 실제 base FINAL_APPROACH
decision을 만들었는지 확인한다.

### 0.20초 이내 reacquire했지만 이동하지 않음

`/leader/supply/detected`, `/leader/base_alignment/state`, `/leader/alignment/command`,
approach enable, velocity guard enable, raw/final Twist를 순서대로 확인한다. Cached TF는
reacquisition이 아니며 source timestamp가 증가한 fresh observation이 필요하다.

### Tag loss 동안 로봇이 움직임

잘못된 동작이다. 즉시 velocity guard를 disable한다. Grace command에 target pose가
없는지, detection이 false인지, stale control target이 새 command로 발행되는지,
raw/final Twist가 zero인지 확인한다.

### ALIGNED인데 gripper가 닫히지 않음

`/gripper_sequence` 실행 여부와 `enabled=true`, `alignment_topic`, base ALIGNED,
`/leader/dynamixel/status`, U2D2 연결 및 `/leader/dynamixel/command`를 확인한다.

### Detection 직후 CLOSE됨

`ros2 param get /gripper_sequence alignment_topic`이
`/leader/base_alignment/state`인지 확인한다. 이전 camera-frame topic 또는 stale ALIGNED
입력을 사용하면 안 된다.

### Gripper가 두 번 동작함

`/sequence/status`, DONE 상태, detection flicker 및 base ALIGNED latch를 확인한다.
DONE은 detection=false와 alignment!=ALIGNED가 동시에 성립해야만 재무장된다.

## 13. 안전한 rollback

먼저 변경 범위를 확인한다.

```bash
cd ~/damgc_robot
git status --short
git diff --stat
git diff
git log --oneline -5
```

변경을 commit한 뒤 되돌려야 한다면 해당 commit을 새 revert commit으로 되돌린다.

```bash
git revert <commit-sha>
```

작업자의 다른 변경까지 삭제할 수 있는 `git reset --hard` 또는 `git clean -fd`는 이
절차에서 사용하지 않는다.
