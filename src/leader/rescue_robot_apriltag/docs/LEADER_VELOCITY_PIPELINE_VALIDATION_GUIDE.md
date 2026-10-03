# Leader base-link velocity command software pipeline 재현·검증 가이드

> 이 문서는 position-only controller를 처음 연결했을 당시의 검증 기록이다.
> 현재 tag-normal target-pose 알고리즘의 authoritative interface와 실차 절차는
> [Leader Tag-Normal 최종 정렬 검증 가이드](LEADER_TAG_NORMAL_ALIGNMENT_VALIDATION_GUIDE.md)를
> 따른다. 아래의 `base_target_forward`/controller `target_forward` 설명은 현행
> 파라미터가 아니라 변경 전 구조를 기록한 것이다.

> **현재 통합 경로 (2026-10-02):** raw approach → velocity guard →
> `/leader/approach/cmd_vel_safe` → Leader Command Selector → `/leader/cmd_vel`이다.
> 이 문서의 과거 final-guard 경로/검증 결과는 당시 기록으로 보존한다. 현행 실행 절차는
> [Leader AprilTag drive run guide](../../rescue_robot_bringup/docs/LEADER_APRILTAG_DRIVE_RUN_GUIDE.md)를 따른다.

## 1. 목적과 범위

이 문서는 Jetson에서 `damgc_robot` repository를 clone 또는 pull한 개발자가
Leader의 AprilTag 기반 velocity command software pipeline을 빌드하고 수동 검증하는
절차를 기록한다.

기존 `/leader/alignment/state`는 D435 camera optical frame 기준 판단이다. 이번 개발은
기존 상태를 유지하면서 다음 경로를 추가했다.

```text
base_link metric
  → base_link alignment state
  → approach controller
  → raw Twist
  → velocity guard
  → final software Twist
```

이 단계의 완료 의미는 **ROS 2 velocity command software pipeline 완료**다. 실제 motor
control, STM32 UART 연결, wheel 구동 및 grasp 동작 완료를 의미하지 않는다.

## 2. 안전 경계와 architecture

```text
D435 RGB + CameraInfo
          ↓
     apriltag_ros
          ↓  TF: camera_color_optical_frame ← leader/tag36h11:0
/leader/apriltag_approach
          ├─→ 기존 camera-frame metric/state
          │     └─→ /leader/alignment/state        (회귀 유지)
          │
          └─→ TF2 exact-stamp transform to base_link
                ├─→ /leader/supply/base_relative_pose
                ├─→ base_forward_distance
                ├─→ base_lateral_error
                ├─→ base_bearing
                └─→ /leader/base_alignment/state
                              ↓
                  /leader/approach_controller
                              ↓
                  /leader/approach/cmd_vel_raw
                              ↓
                    /leader/velocity_guard
                              ↓
                       /leader/cmd_vel
                              X
                    STM32 bridge / Motor
                 (이번 단계에서는 미연결)
```

Perception node는 `/leader/cmd_vel`을 발행하지 않는다. Controller는 UART를 다루지
않으며 raw command만 발행한다. 최종 software topic은 독립 guard만 발행한다.

## 3. 구현 파일

아래 표는 이 기능을 위해 현재 작업 트리에서 변경되거나 추가된 파일이다.

| 경로 | 역할 | 변경 내용 |
|---|---|---|
| `src/leader/rescue_robot_apriltag/CMakeLists.txt` | perception package build/test | base state test 등록 |
| `src/leader/rescue_robot_apriltag/config/approach.yaml` | perception 설정 | provisional base target/tolerance/stability parameter 추가 |
| `src/leader/rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py` | perception ROS node | base pose와 같은 cycle에서 base metric/state 발행, lost 처리 추가 |
| `src/leader/rescue_robot_apriltag/rescue_robot_apriltag/base_alignment_logic.py` | ROS 비의존 decision logic | base state priority, freshness 및 stable-time 구현 |
| `src/leader/rescue_robot_apriltag/test/test_base_pose.py` | base pose regression | node publisher와 base output/lost 동작 검증 보강 |
| `src/leader/rescue_robot_apriltag/test/test_base_alignment_logic.py` | base state test | 상태, 경계, stale, NaN/inf 및 안정 시간 시험 |
| `src/leader/leader_approach_control/package.xml` | control package manifest | ROS message, launch, service runtime/test dependency 정의 |
| `src/leader/leader_approach_control/setup.py` | Python package 설치 | controller/guard executable, YAML 및 launch 설치 |
| `src/leader/leader_approach_control/setup.cfg` | Python executable 설치 설정 | ROS executable 경로 설정 |
| `src/leader/leader_approach_control/resource/leader_approach_control` | ament index | package resource 등록 |
| `src/leader/leader_approach_control/leader_approach_control/__init__.py` | Python package | package 초기화 |
| `src/leader/leader_approach_control/leader_approach_control/approach_controller_logic.py` | ROS 비의존 controller | state gate, 연속 오차식, saturation 및 freshness/coherence 검사 |
| `src/leader/leader_approach_control/leader_approach_control/approach_controller_node.py` | raw controller node | 입력 sample 결합, enable service, watchdog 및 raw Twist 발행 |
| `src/leader/leader_approach_control/leader_approach_control/velocity_guard_logic.py` | ROS 비의존 guard | finite/axis/reverse 검사, clamp, freshness 및 slew 계산 |
| `src/leader/leader_approach_control/leader_approach_control/velocity_guard_node.py` | final guard node | enable gate, timeout, invalid reject, final Twist 및 shutdown zero |
| `src/leader/leader_approach_control/config/approach_controller.yaml` | controller 설정 | gain, raw limit, timeout 및 startup gate |
| `src/leader/leader_approach_control/config/velocity_guard.yaml` | guard 설정 | final limit, acceleration, timeout 및 startup gate |
| `src/leader/leader_approach_control/launch/approach_controller.launch.py` | controller launch | `/leader/approach_controller` 단독 실행 |
| `src/leader/leader_approach_control/launch/velocity_guard.launch.py` | guard launch | `/leader/velocity_guard` 단독 실행 |
| `src/leader/leader_approach_control/test/test_approach_controller_logic.py` | controller unit test | state별 command, gate, saturation, stale 및 invalid 시험 |
| `src/leader/leader_approach_control/test/test_approach_controller_node.py` | controller node helper test | Twist 축과 pose validation 시험 |
| `src/leader/leader_approach_control/test/test_velocity_guard_logic.py` | guard unit test | gate, clamp, invalid, timeout, slew 및 shutdown 시험 |
| `src/leader/leader_approach_control/README.md` | package 요약 | interface, build 및 기본 실행 절차 |
| `src/leader/rescue_robot_apriltag/docs/LEADER_VELOCITY_PIPELINE_VALIDATION_GUIDE.md` | 상세 검증 문서 | 본 재현·검증 가이드 |
| `docs/progress/week 1/README.md` | 진행 상황 index | software pipeline 완료 상태와 본 가이드 링크 추가 |

## 4. ROS 2 interface

### 4.1 Topics

| Topic | Type | Publisher | Subscriber/용도 |
|---|---|---|---|
| `/leader/supply/detected` | `std_msgs/msg/Bool` | `/leader/apriltag_approach` | controller detection gate |
| `/leader/supply/tag_id` | `std_msgs/msg/Int32` | `/leader/apriltag_approach` | controller selected-ID gate; lost는 `-1` |
| `/leader/supply/base_relative_pose` | `geometry_msgs/msg/PoseStamped` | `/leader/apriltag_approach` | controller의 authoritative stamped base sample |
| `/leader/supply/base_forward_distance` | `std_msgs/msg/Float64` | `/leader/apriltag_approach` | 사람이 확인하는 base +X 전방 거리 `[m]` |
| `/leader/supply/base_lateral_error` | `std_msgs/msg/Float64` | `/leader/apriltag_approach` | 사람이 확인하는 base +Y 좌측 오차 `[m]` |
| `/leader/supply/base_bearing` | `std_msgs/msg/Float64` | `/leader/apriltag_approach` | 사람이 확인하는 `atan2(y, x)` `[rad]` |
| `/leader/alignment/state` | `std_msgs/msg/String` | `/leader/apriltag_approach` | 기존 camera-frame state; 회귀 비교용 |
| `/leader/base_alignment/state` | `std_msgs/msg/String` | `/leader/apriltag_approach` | controller의 high-level base state 입력 |
| `/leader/approach/cmd_vel_raw` | `geometry_msgs/msg/Twist` | `/leader/approach_controller` | `/leader/velocity_guard` 입력 |
| `/leader/cmd_vel` | `geometry_msgs/msg/Twist` | `/leader/velocity_guard` | 이번 단계의 최종 software output |

세 개의 header 없는 base metric topic은 관찰용이다. Controller는 서로 다른 callback의
Float64 값을 조합하지 않고, 하나의 stamped `base_relative_pose`에서 forward=`x`,
lateral=`y`, bearing=`atan2(y,x)`를 함께 다시 계산한다.

### 4.2 Services와 nodes

| Interface | Type | 의미 |
|---|---|---|
| `/leader/approach/enable` | `std_srvs/srv/SetBool` | controller enable/disable; 전환 시 기존 sample 폐기 및 zero 발행 |
| `/leader/velocity_guard/enable` | `std_srvs/srv/SetBool` | guard enable/disable; 전환 시 command cache 폐기 및 zero 발행 |

실제 node 이름은 `/leader/apriltag_approach`, `/leader/approach_controller`,
`/leader/velocity_guard`다. 두 enable 값은 startup parameter이지만 runtime 전환은 위
service를 사용한다. `ros2 param set`으로 startup parameter를 바꾸는 절차가 아니다.

## 5. Base alignment state 설계

좌표축은 `base_link`의 `+X=전방`, `+Y=왼쪽`, `+Z=위`다. 기본값에서 bearing tolerance는
`5 deg`, target 구간은 `0.25 ± 0.03 m`, lateral tolerance는 `±0.02 m`다.

상태 priority는 아래 표의 위에서 아래 순서다. tolerance 경계값 자체는 tolerance 안으로
처리한다.

| 우선순위/상태 | 실제 조건 | 의미 | Controller 처리 |
|---|---|---|---|
| 1 `TAG_LOST` | sample 없음, invalid ID/값, non-forward, stale 또는 TF 실패 | 안전하게 판단할 base sample 없음 | 즉시 raw zero |
| 2 `TURN_LEFT` | `bearing > +bearing_tolerance` | 태그가 로봇 진행축 왼쪽 | `linear.x=0`, `angular.z>0` |
| 3 `TURN_RIGHT` | `bearing < -bearing_tolerance` | 태그가 로봇 진행축 오른쪽 | `linear.x=0`, `angular.z<0` |
| 4 `APPROACH` | bearing 허용, `forward > target+tolerance` | 방향이 맞고 목표 구간보다 멂 | 양의 `linear.x`와 작은 angular correction |
| 5 `TOO_CLOSE` | bearing 허용, `forward < target-tolerance` | 목표 구간보다 가까움 | 현재 정책은 후진하지 않고 zero |
| 6 `FINE_ALIGN_LEFT` | forward/bearing 허용, `lateral > +tolerance` | 목표 거리에서 왼쪽 오차 잔존 | 차동구동이므로 `linear.y` 없이 양의 회전만 사용 |
| 7 `FINE_ALIGN_RIGHT` | forward/bearing 허용, `lateral < -tolerance` | 목표 거리에서 오른쪽 오차 잔존 | 차동구동이므로 `linear.y` 없이 음의 회전만 사용 |
| 8 `STABILIZING` | 모든 tolerance 안에 진입했으나 `stable_time` 미만 | 연속 안정성 확인 중 | raw zero |
| 9 `ALIGNED` | 모든 tolerance를 `stable_time` 동안 연속 유지 | software alignment 완료 | raw zero |

Tolerance 밖으로 나가거나 tag ID가 변경되거나 tag가 lost되면 안정화 timer는 reset된다.

## 6. Provisional target 경고

> **WARNING — 실제 파지 거리 아님**
>
> `base_target_forward=0.25 m`, `target_forward=0.25 m`와 모든 base tolerance는
> software/state/topic 검증용 provisional 값이다. 최종 gripper grasp 거리, TCP 목표,
> 접촉 거리 또는 motor stopping distance가 아니다.

기존 camera optical frame의 `target_distance=0.15 m`를 base target으로 복사하지 않았다.
Camera origin과 `base_link` origin이 다르므로 같은 수치가 같은 물리 위치를 뜻하지 않는다.
향후 `gripper_approach_link` 또는 gripper TCP extrinsic을 실측·calibration한 뒤
`base_target_forward`와 controller `target_forward`를 같은 값으로 다시 조정해야 한다.

## 7. Controller 계산과 sample gate

State는 동작 종류를 고르고, 연속 base 오차는 속도 크기를 정한다.

```text
forward_error = base_forward_distance - target_forward

v_candidate = linear_gain * forward_error
w_candidate = angular_gain * base_bearing
              + lateral_gain * base_lateral_error
```

- `APPROACH`에서 `v_candidate`를 사용한다. `allow_reverse=false`이면 음수는 0으로 만든 후
  `±max_raw_linear_speed`로 clamp한다.
- `APPROACH`와 `FINE_ALIGN_*`의 angular correction은 `w_candidate`를
  `±max_raw_angular_speed`로 clamp한다.
- `TURN_LEFT/RIGHT`는 lateral term 없이 `angular_gain * bearing`만 사용하고 잘못된
  sign이면 zero로 fail closed한다.
- `FINE_ALIGN_LEFT/RIGHT`도 state와 계산된 angular sign이 일치할 때만 회전한다.
- 모든 Twist는 차동구동 축인 `linear.x`와 `angular.z`만 채운다.

Non-zero raw command에는 다음 조건이 모두 필요하다.

1. controller enabled
2. `/leader/supply/detected=true`
3. selected `tag_id=target_tag_id`
4. `base_relative_pose.header.frame_id=base_link`
5. 유효하고 forward `x>0`인 finite pose와 quaternion
6. source stamp와 local receipt age가 `controller_pose_timeout` 이내
7. base state가 pose 뒤 `sample_sync_tolerance` 이내에 도착
8. 알려진 non-stop state와 유효한 연속 오차

Enable 전환, detected/tag ID 전환, stale, unknown state 또는 incoherent sample은 cache를
폐기하고 raw zero를 만든다.

## 8. Velocity Guard 동작

Guard는 raw controller와 분리된 `/leader/velocity_guard` node다.

| 기능 | 실제 동작 |
|---|---|
| enabled gate | startup은 false. Disabled이면 final zero이며 enable 시 fresh raw를 새로 기다림 |
| finite 검사 | Twist 6축 중 하나라도 NaN/inf이면 command 전체 reject 및 즉시 zero |
| 허용 축 검사 | `linear.y/z`, `angular.x/y`가 `axis_epsilon`을 넘으면 전체 reject 및 즉시 zero |
| reverse policy | `allow_reverse=false`에서 `linear.x<0`이면 전체 reject 및 즉시 zero |
| speed clamp | finite planar input을 `max_linear_speed`, `max_angular_speed`로 clamp |
| slew limit | 이전 final에서 candidate까지 linear/angular acceleration 양방향 제한 |
| elapsed-time cap | 한 update의 slew 계산 `dt`를 `max_slew_dt` 이하로 제한 |
| timeout | 마지막 valid raw receipt 후 `command_timeout` 초가 지나면 즉시 zero |
| publisher loss | 새 raw가 없어 timeout이 되면 final zero를 계속 발행 |
| invalid raw | cache에 저장하지 않고 이전 output도 즉시 zero로 reset |
| shutdown | 정상 종료 경로에서 `shutdown_stop_count`회의 zero burst 발행 |

정상적인 valid raw가 non-zero에서 zero로 바뀔 때는 감속 slew가 적용된다. 예를 들어 최대
`0.05 m/s`에서 `0.10 m/s²`로 감속하면 최대 약 `0.5 s`가 필요하다. 반면 disabled,
invalid, timeout은 safety transition이므로 즉시 zero다.

## 9. Parameter reference

### 9.1 `/leader/apriltag_approach`

Source: `src/leader/rescue_robot_apriltag/config/approach.yaml`

| Parameter | Default | Unit | 의미 | 구분 |
|---|---:|---|---|---|
| `source_frame` | `camera_color_optical_frame` | frame | tag TF source frame | 설치 구성 |
| `base_frame` | `base_link` | frame | base output target frame | 설치 구성 |
| `tf_lookup_timeout` | `0.0` | s | base TF lookup wait | 시험 구성 |
| `tag_frame_pattern` | `leader/tag36h11:{id}` | frame pattern | tag ID별 TF frame | interface |
| `target_tag_id` | `0` | ID | 고정 target; `-1`이면 multi-tag mode | mission 설정 |
| `allowed_tag_ids` | `[0,1,2]` | ID list | multi-tag 후보 | mission 설정 |
| `selection_mode` | `priority` | - | `priority` 또는 `nearest` | mission 설정 |
| `target_distance` | `0.15` | m | 기존 camera-frame 목표 | tentative, grasp 아님 |
| `distance_tolerance` | `0.02` | m | camera distance tolerance | tentative |
| `lateral_tolerance` | `0.02` | m | camera lateral tolerance | tentative |
| `angle_tolerance_deg` | `5.0` | deg | camera angle tolerance | tentative |
| `tag_timeout` | `1.0` | s | tag/base sample stale timeout | 측정 기반 trial |
| `stable_time` | `0.8` | s | camera state 안정 유지 시간 | tentative |
| `publish_rate` | `20.0` | Hz | perception/state update rate | software 설정 |
| `filter_window` | `5` | samples | translation median window | software 설정 |
| `base_target_forward` | `0.25` | m | base state 목표 전방 거리 | provisional |
| `base_forward_tolerance` | `0.03` | m | base forward tolerance | provisional |
| `base_lateral_tolerance` | `0.02` | m | base lateral tolerance | provisional |
| `base_bearing_tolerance_deg` | `5.0` | deg | base bearing tolerance | provisional |
| `base_stable_time` | `0.8` | s | base tolerance 연속 유지 시간 | provisional |

### 9.2 `/leader/approach_controller`

Source: `src/leader/leader_approach_control/config/approach_controller.yaml`

| Parameter | Default | Unit | 의미 | 구분 |
|---|---:|---|---|---|
| `base_frame` | `base_link` | frame | 허용 pose frame | interface |
| `target_tag_id` | `0` | ID | 허용 selected tag | mission 설정 |
| `controller_enabled_on_startup` | `false` | bool | startup raw gate | safety default |
| `controller_publish_rate` | `20.0` | Hz | raw zero/non-zero 발행 주기 | software 설정 |
| `controller_pose_timeout` | `0.35` | s | pose source/receipt freshness | tentative safety |
| `sample_sync_tolerance` | `0.10` | s | pose 뒤 state 수신 허용 시간 | tentative safety |
| `target_forward` | `0.25` | m | forward error 목표 | provisional, grasp 아님 |
| `linear_gain` | `0.20` | `(m/s)/m` | forward proportional gain | tentative tuning |
| `angular_gain` | `0.80` | `(rad/s)/rad` | bearing proportional gain | tentative tuning |
| `lateral_gain` | `0.50` | `(rad/s)/m` | lateral angular correction gain | tentative tuning |
| `max_raw_linear_speed` | `0.05` | m/s | raw candidate saturation | tentative software limit |
| `max_raw_angular_speed` | `0.20` | rad/s | raw candidate saturation | tentative software limit |
| `allow_reverse` | `false` | bool | controller reverse 허용 | current safety policy |

`base_target_forward`와 controller `target_forward`는 항상 같은 값으로 변경한다.

### 9.3 `/leader/velocity_guard`

Source: `src/leader/leader_approach_control/config/velocity_guard.yaml`

| Parameter | Default | Unit | 의미 | 구분 |
|---|---:|---|---|---|
| `guard_enabled_on_startup` | `false` | bool | startup final gate | safety default |
| `publish_rate` | `50.0` | Hz | final command 발행 주기 | software 설정 |
| `command_timeout` | `0.30` | s | 마지막 valid raw local receipt timeout | tentative safety |
| `max_linear_speed` | `0.05` | m/s | final `linear.x` clamp | tentative, motor tuning 전 |
| `max_angular_speed` | `0.20` | rad/s | final `angular.z` clamp | tentative, motor tuning 전 |
| `max_linear_acceleration` | `0.10` | m/s² | linear slew limit | tentative, motor tuning 전 |
| `max_angular_acceleration` | `0.40` | rad/s² | angular slew limit | tentative, motor tuning 전 |
| `max_slew_dt` | `0.10` | s | 한 update의 slew 계산 최대 dt | software safety |
| `axis_epsilon` | `1.0e-9` | SI axis unit | unused axis 허용 수치 오차 | software safety |
| `allow_reverse` | `false` | bool | negative `linear.x` 허용 | current safety policy |
| `shutdown_stop_count` | `3` | messages | 정상 종료 zero burst 수 | software safety |
| `command_topic` | `/leader/approach/cmd_vel_raw` | topic | guard input | interface |
| `safe_command_topic` | `/leader/cmd_vel` | topic | guard output | interface |

## 10. Jetson build

현재 Jetson에 ROS 2 Humble, RealSense ROS, `apriltag_ros`와 repository의 기존 dependency가
설치되어 있다고 가정한다. 저장소 root 이름이 다르면 첫 줄의 경로만 바꾼다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  rescue_robot_description \
  rescue_robot_apriltag \
  rescue_robot_bringup \
  leader_approach_control

source install/local_setup.bash
```

전체 workspace build는 이 기능 검증에 필요하지 않다. Repository에는 별도 STM32 DAMGC
firmware와 다른 대형 perception package가 있으므로, 관련 ROS package만 선택하면
firmware/toolchain 또는 unrelated dependency 문제를 피할 수 있다.

## 11. 자동 테스트

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

colcon test \
  --packages-select rescue_robot_apriltag leader_approach_control \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/rescue_robot_apriltag/test_results \
  --verbose

colcon test-result \
  --test-result-base build/leader_approach_control \
  --verbose
```

2026-08-29 현재 실제 실행 결과:

| Package | Total | Passed | Failed | Errors | 구성 |
|---|---:|---:|---:|---:|---|
| `rescue_robot_apriltag` | 108 | 108 | 0 | 0 | 기존 camera 46 + base pose 29 + base state 33 |
| `leader_approach_control` | 60 | 60 | 0 | 0 | controller 35 + guard 25 |
| 합계 | 168 | 168 | 0 | 0 | 관련 regression 전체 |

## 12. 수동 검증 전 안전 조건

> **WARNING — 아래 조건을 모두 확인하기 전 guard를 enable하지 않는다.**

- STM32 bridge를 실행하지 않는다.
- Motor power는 OFF를 권장한다.
- `/leader/cmd_vel`이 실제 hardware subscriber로 연결되지 않았는지 확인한다.
- Controller와 guard는 기본값 `enabled=false`로 시작한다.
- `/leader/cmd_vel` publisher는 guard 하나만 허용한다.
- `arrow_key_teleop`은 같은 topic의 별도 publisher이므로 실행하지 않는다.
- `leader_cooperation`은 `/leader/cmd_vel` subscriber이므로 이번 검증에서는 실행하지 않는다.

모든 test launch 전에 다음을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash

ros2 node list
ros2 node list | grep -E 'stm32_bridge|arrow_key_teleop|leader_cooperation' || true
ros2 topic info /leader/cmd_vel --verbose
```

두 번째 명령은 아무 node도 출력하지 않아야 한다. Guard를 아직 실행하지 않았다면 마지막
명령의 `Unknown topic`은 정상이다. Guard를 실행한 뒤에는 publisher가 정확히 하나이고
node가 `/leader/velocity_guard`여야 한다. `ros2 topic echo`를 실행 중이면 echo node가
subscriber로 잠시 표시될 수 있다.

`/leader/stm32_bridge`가 보이면 해당 launch terminal에서 `Ctrl+C`로 종료한다. 이 문서의
검증 중에는 `ros2 launch stm32_bridge ...`를 실행하지 않는다.

## 13. Terminal 1 — Camera / AprilTag / base perception

실제 launch argument는 `enable_depth`, `enable_infra`, `enable_imu`, `enable_approach`,
`approach_config`다. RGB AprilTag와 approach node만 필요한 최소 실행은 다음과 같다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=false \
  enable_infra:=false \
  enable_imu:=false \
  enable_approach:=true
```

Tag family는 config의 `tag36h11`, 기본 target은 ID 0이다. 이 launch는 camera,
rectification, `apriltag_ros`, `robot_state_publisher`와
`/leader/apriltag_approach`를 실행하지만 controller/guard/STM32 bridge는 실행하지 않는다.

## 14. Terminal 2 — Base pose와 metric 확인

새 terminal마다 먼저 다음 두 줄을 실행한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Pose와 metric을 한 번에 하나씩 확인한다. 계속 관찰하려면 `--once`를 제거한다.

```bash
ros2 topic echo /leader/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once

ros2 topic echo /leader/supply/base_forward_distance \
  std_msgs/msg/Float64 --once

ros2 topic echo /leader/supply/base_lateral_error \
  std_msgs/msg/Float64 --once

ros2 topic echo /leader/supply/base_bearing \
  std_msgs/msg/Float64 --once
```

| 실제 tag 위치 | forward | lateral | bearing |
|---|---|---|---|
| 정면 | 양수 | 약 0 | 약 0 |
| 로봇 왼쪽 | 양수 | `>0` | `>0` |
| 로봇 오른쪽 | 양수 | `<0` | `<0` |
| 더 멀리 | 증가 | 위치에 따른 sign | 위치에 따른 sign |

물리 D435/base metric의 기존 실측 절차와 결과는
[`LEADER_BASE_LINK_POSE_METRICS_VALIDATION_GUIDE.md`](../../rescue_robot_apriltag/docs/LEADER_BASE_LINK_POSE_METRICS_VALIDATION_GUIDE.md)를 참고한다.

## 15. Terminal 3 — Base state와 기존 camera state 비교

```bash
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String
```

다른 terminal 또는 tmux pane에서 기존 camera state도 함께 본다.

```bash
ros2 topic echo /leader/alignment/state std_msgs/msg/String
```

두 state는 좌표계, 목표 거리 및 tolerance가 다르므로 항상 같은 문자열이어야 하는 것은
아니다. 기존 camera state가 계속 발행되고 tag hidden에서 `TAG_LOST`가 되는지를 회귀
확인한다.

| Tag 위치/조건 | 기대 base state |
|---|---|
| bearing tolerance보다 왼쪽 | `TURN_LEFT` |
| bearing tolerance보다 오른쪽 | `TURN_RIGHT` |
| 방향이 맞고 `forward>0.28 m` | `APPROACH` |
| 방향이 맞고 `forward<0.22 m` | `TOO_CLOSE` |
| 목표 거리에서 lateral이 왼쪽/오른쪽 | `FINE_ALIGN_LEFT` / `FINE_ALIGN_RIGHT` |
| 모든 tolerance에 처음 진입 | `STABILIZING` |
| 모든 tolerance를 0.8 s 연속 유지 | `ALIGNED` |
| Tag hidden/stale/TF fault | `TAG_LOST` |

## 16. Terminal 4 — Approach controller와 raw command

Controller process용 새 terminal:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch leader_approach_control approach_controller.launch.py
```

다른 terminal에서 startup zero를 먼저 확인하고 service로 enable한다.

```bash
ros2 topic echo /leader/approach/cmd_vel_raw \
  geometry_msgs/msg/Twist --once

ros2 service call /leader/approach/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /leader/approach/cmd_vel_raw geometry_msgs/msg/Twist
```

| Base state | 기대 raw command |
|---|---|
| `TURN_LEFT` | `linear.x=0`, `angular.z>0` |
| `TURN_RIGHT` | `linear.x=0`, `angular.z<0` |
| `APPROACH` | `linear.x>0`, 작은 bearing/lateral angular correction 가능 |
| `FINE_ALIGN_LEFT/RIGHT` | `linear.x=0`, 해당 sign의 `angular.z` |
| `TAG_LOST`, `TOO_CLOSE`, `STABILIZING`, `ALIGNED` | 모든 축 zero |

## 17. Terminal 5 — Guard startup disabled 확인

Guard process용 새 terminal:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch leader_approach_control velocity_guard.launch.py
```

Guard를 enable하지 않은 상태에서 raw가 non-zero인 tag 위치를 만든다. 다른 terminal에서
다음을 확인한다.

```bash
ros2 topic echo /leader/approach/cmd_vel_raw \
  geometry_msgs/msg/Twist --once

ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist --once
```

Raw가 non-zero여도 final의 6축은 모두 zero여야 한다.

## 18. Terminal 6 — Guard enable 후 final command

Enable 직전에 publisher/subscriber를 다시 확인한다.

```bash
ros2 node list | grep -E 'stm32_bridge|arrow_key_teleop|leader_cooperation' || true
ros2 topic info /leader/cmd_vel --verbose
```

첫 명령에 아무것도 나오지 않고 final publisher가 `/leader/velocity_guard` 하나인 경우에만
계속한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist
```

| Base state | 기대 final command |
|---|---|
| `TURN_LEFT` | slew 범위 안에서 `angular.z>0` |
| `TURN_RIGHT` | slew 범위 안에서 `angular.z<0` |
| `APPROACH` | `0<linear.x≤0.05 m/s`, `|angular.z|≤0.20 rad/s` |
| `FINE_ALIGN_LEFT/RIGHT` | `linear.x=0`, 제한된 해당 sign angular |
| `STABILIZING`, `ALIGNED` | raw zero; 정상 감속 slew 후 final zero |
| `TAG_LOST`, invalid, timeout, disabled | final 즉시 zero |

## 19. Command timeout / publisher loss 시험

정상 `Ctrl+C` 종료는 controller가 zero를 한 번 발행한 후 종료하므로 graceful stop을
검증한다. `command_timeout=0.30 s` watchdog 자체를 검증하려면 motor/STM32가 없는 상태에서
controller process만 비정상 종료시킨다.

1. `APPROACH`에서 final `linear.x>0`을 확인한다.
2. Controller terminal의 PID를 확인한다.

```bash
pgrep -af 'approach_controller_node'
```

3. 출력된 command와 PID가 controller 하나임을 눈으로 확인한 뒤 그 **정확한 PID만**
   종료한다.

```bash
kill -KILL <확인한_CONTROLLER_PID>
```

4. 별도 terminal에서 final을 계속 관찰한다.

```bash
ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist
```

마지막 valid raw 수신 후 `0.30 s`와 guard publish period 약 `0.02 s` 이내에 final이
zero가 되고 계속 zero여야 한다. 시험 후 controller launch를 다시 시작하고 service로
enable해야 한다.

## 20. Guard test-only input

이 시험 동안 production controller를 종료하고 `/leader/approach/cmd_vel_raw` publisher가
없음을 먼저 확인한다.

```bash
ros2 topic info /leader/approach/cmd_vel_raw --verbose
```

Guard는 enabled 상태로 둔다. Over-limit 입력을 20 Hz로 주입하고 다른 terminal에서 final을
본다.

```bash
ros2 topic pub --rate 20 \
  /leader/approach/cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}"
```

Slew ramp 뒤 final은 `linear.x=0.05`, `angular.z=0.20`을 넘지 않아야 한다. `Ctrl+C`로
publisher를 중단하면 timeout 후 zero가 된다.

Reverse reject 시험:

```bash
ros2 topic pub --rate 20 \
  /leader/approach/cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: -0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

`allow_reverse=false`이므로 final은 zero이고 guard log에 reject warning이 발생한다.

NaN/inf와 unused-axis fault는 YAML/CLI parser 및 middleware별 표현 차이 때문에 억지로
수동 명령을 만들지 않는다. 이 항목은 `test_velocity_guard_logic.py` 자동 test와
software-only test publisher로 검증했다.

## 21. 수행 결과 기록

아래 값은 2026-08-29 production perception/controller/guard node에 동적 test TF를 넣은
software-only 통합 시험 결과다. 실제 D435 광학 검출이나 motor 출력 결과가 아니다.

| Test | Base state | Raw linear | Raw angular | Final linear | Final angular | Expected | Result |
|---|---|---:|---:|---:|---:|---|---|
| LEFT | `TURN_LEFT` | 0.000 | +0.132 | 0.000 | +0.132 | +angular | PASS |
| RIGHT | `TURN_RIGHT` | 0.000 | -0.132 | 0.000 | -0.115 | -angular | PASS |
| FAR | `APPROACH` | +0.040 | 0.000 | +0.040 | 0.000 | +linear, within clamp | PASS |
| TARGET/STABILIZING | `STABILIZING` | 0.000 | 0.000 | 0.000 | 0.000 | zero | PASS |
| TARGET/ALIGNED | `ALIGNED` | 0.000 | 0.000 | 0.000 | 0.000 | zero | PASS |
| HIDDEN | `TAG_LOST` | 0.000 | 0.000 | 0.000 | 0.000 | zero | PASS |
| CONTROLLER_DISABLED | `TURN_LEFT` | 0.000 | 0.000 | 0.000 | 0.000 | raw/final zero | PASS |
| GUARD_DISABLED | `APPROACH` | +0.040 | 0.000 | 0.000 | 0.000 | final zero | PASS |
| CONTROLLER_KILL | `APPROACH` | last +0.040 | last 0.000 | 0.000 | 0.000 | timeout→zero | PASS |
| NaN / inf | - | invalid | - | 0.000 | 0.000 | reject/zero | PASS |
| Over-limit | - | +1.000 | +2.000 | +0.050 | +0.200 | clamp | PASS |
| Reverse | - | -0.020 | 0.000 | 0.000 | 0.000 | reject/zero | PASS |

각 scenario에서 `/leader/cmd_vel` publisher는 `/leader/velocity_guard` 하나였다. 실제 D435로
LEFT/RIGHT/FAR/TARGET를 다시 움직이며 controller와 final까지 확인하는 시험은
**NOT VERIFIED**이며, 이 문서의 Terminal 1~6 절차로 Jetson에서 수행해야 한다. 기존 D435
base pose/metric sign은 별도 linked guide에서 실측 완료되어 있다.

## 22. 문제 해결

### Base state가 나오지 않음

```bash
ros2 node list
ros2 topic list | sort
ros2 topic echo /leader/supply/detected std_msgs/msg/Bool --once
ros2 topic echo /leader/supply/tag_id std_msgs/msg/Int32 --once
ros2 topic info /leader/base_alignment/state --verbose
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame 'leader/tag36h11:0'
```

`enable_approach:=true`, ID 0 TF, `base_link→camera_color_optical_frame` TF와 source timestamp를
순서대로 확인한다.

### `cmd_vel_raw` topic이 없거나 항상 zero

```bash
ros2 node list | grep approach_controller
ros2 topic info /leader/approach/cmd_vel_raw --verbose
ros2 topic echo /leader/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String --once
ros2 param list /leader/approach_controller
ros2 param get /leader/approach_controller controller_pose_timeout
ros2 param get /leader/approach_controller sample_sync_tolerance
```

그 다음 enable service를 다시 호출한다. Enabled여도 detected=false, tag ID 불일치, stale pose,
pose/state 순서 불일치, stop state이면 zero가 정상이다.

### `/leader/cmd_vel`이 항상 zero 또는 non-zero가 되지 않음

```bash
ros2 topic echo /leader/approach/cmd_vel_raw \
  geometry_msgs/msg/Twist --once
ros2 topic info /leader/cmd_vel --verbose
ros2 param list /leader/velocity_guard
ros2 param get /leader/velocity_guard command_timeout
ros2 param get /leader/velocity_guard allow_reverse
```

Guard enable service 응답이 success인지 확인한다. Enable 직후에는 이전 raw를 재사용하지
않고 fresh raw를 기다린다. Raw가 zero이면 final도 zero다.

### `/leader/cmd_vel` publisher가 여러 개

```bash
ros2 topic info /leader/cmd_vel --verbose
ros2 node list | grep -E 'velocity_guard|arrow_key_teleop|stm32_bridge|leader_cooperation'
```

`arrow_key_teleop`은 별도 publisher이므로 종료한다. `stm32_bridge`와 `leader_cooperation`은
subscriber지만 이번 검증에서는 hardware/외부 전달 경계를 막기 위해 함께 종료한다. Mux가
없으므로 여러 publisher를 동시에 실행해도 ROS 2가 priority를 정해주지 않는다.

### Publisher 중단 후 zero가 되지 않음

```bash
ros2 topic info /leader/approach/cmd_vel_raw --verbose
ros2 param get /leader/velocity_guard command_timeout
ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist
```

Test publisher까지 완전히 종료됐는지 확인한다. Valid raw publisher가 하나라도 계속
발행하면 timeout은 발생하지 않는다.

### 회전 sign이 반대임

```bash
ros2 topic echo /leader/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 topic echo /leader/supply/base_lateral_error \
  std_msgs/msg/Float64 --once
ros2 topic echo /leader/supply/base_bearing \
  std_msgs/msg/Float64 --once
ros2 topic echo /leader/base_alignment/state std_msgs/msg/String --once
```

Tag가 로봇 왼쪽이면 base `y`, lateral, bearing이 모두 양수이고 `TURN_LEFT`의 angular.z가
양수여야 한다. Camera state와 base state 문자열이 다르다는 이유만으로 sign을 뒤집지
않는다. TF가 실제 장착 방향과 다르면 이 control 문서 범위에서 임의로 extrinsic을
수정하지 말고 기존 base pose validation 절차로 원인을 분리한다.

## 23. PASS 기준

- base pose/metric과 base state가 fresh tag에서 정상 발행된다.
- LEFT는 양의 bearing/state/raw/final angular, RIGHT는 음의 sign이다.
- FAR에서 `APPROACH`와 양의 raw/final linear가 나온다.
- Controller disabled이면 raw zero다.
- Guard disabled이면 raw가 non-zero여도 final zero다.
- Lost, aligned, invalid 및 stale에서 안전하게 zero가 된다.
- Raw publisher loss 후 timeout으로 final zero가 된다.
- Over-limit 입력은 final limit로 clamp된다.
- Existing camera-frame state와 46개 camera regression test가 유지된다.
- Test 환경의 `/leader/cmd_vel` publisher는 guard 하나뿐이다.
- STM32 bridge, UART 및 motor가 연결되지 않는다.

## 24. 아직 하지 않은 작업

- [ ] `gripper_approach_link` 또는 gripper TCP calibration
- [ ] 최종 `target_forward`와 tolerance 결정
- [ ] 실제 motor-safe velocity/acceleration tuning
- [ ] STM32 bridge integration
- [ ] Motor power OFF 상태 wheel air test
- [ ] Ground manual low-speed test
- [ ] AprilTag 기반 motor rotation sign test
- [ ] Automatic low-speed approach
- [ ] UART fault injection
- [ ] Gripper controller
- [ ] Grasp 위치와 힘 검증
- [ ] Automatic grasp sequence

## 25. 종료 및 작업 트리 확인

시험이 끝나면 controller와 guard를 먼저 disable한다.

```bash
ros2 service call /leader/approach/enable \
  std_srvs/srv/SetBool "{data: false}"

ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

각 launch terminal을 `Ctrl+C`로 종료하고 다음을 확인한다.

```bash
cd ~/damgc_robot
git status
git diff
```

문서의 명령은 STM32 bridge나 motor를 시작하지 않는다. 다음 단계에서 hardware를 연결할
때는 별도 승인된 motor-safe bringup, E-stop 및 mux 정책을 먼저 정의해야 한다.
