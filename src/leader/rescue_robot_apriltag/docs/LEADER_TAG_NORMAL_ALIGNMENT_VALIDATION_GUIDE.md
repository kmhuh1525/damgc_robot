# Leader Tag-Normal 최종 정렬 검증 가이드

> **Historical / superseded:** 이 문서는 먼 거리부터 pre-align point를 직접 추종했던
> 첫 orientation 기반 구현을 기록한다. 실차에서 과도한 회전, FOV 이탈, `TAG_LOST`,
> 지그재그가 확인되어 현재는 hybrid 방식으로 대체됐다. 현재 설계와 검증 절차는
> [Hybrid algorithm](LEADER_HYBRID_TAG_ALIGNMENT_ALGORITHM.md) 및
> [Hybrid validation guide](LEADER_HYBRID_TAG_ALIGNMENT_VALIDATION_GUIDE.md)를 사용한다.

## 1. 목적과 알고리즘

기존 알고리즘은 태그 중심의 `base_forward_distance`, `base_lateral_error`,
`base_bearing`만 사용했다. 로봇이 태그 정면 중심선 밖에서 태그 중심을 바라보면 세 값이
정상처럼 보여 잘못 ALIGNED가 될 수 있었다.

새 알고리즘은 `apriltag_ros`가 제공한 tag pose orientation으로 tag frame `+Z`를
`base_link`에 회전한 뒤 XY 평면에 projection한다. Leader 실차의 `apriltag_ros` TF를
검증한 결과, 정면에서 본 tag `+Z`는 인쇄면에서 관측 로봇 쪽으로 나오는 outward
normal이다. 로봇의 최종 진행 방향은 그 반대인 `-Z`다.

```text
             TAG
              ■
              │  tag outward normal (+Z, toward robot)
       0.20 m X  final target pose
              │
       0.30 m O  pre-align target pose
              │
            ROBOT
```

```text
tag_outward_normal = project_xy(R_base_tag * tag_Z)
pre_target         = tag_position + 0.30 * tag_outward_normal
final_target       = tag_position + 0.20 * tag_outward_normal
target_yaw         = heading(-tag_outward_normal)
```

Raw quaternion에서 camera-frame planar yaw를 직접 추출하지 않는다. Projected normal이
퇴화하거나 `tag_outward_normal · tag_position >= 0`인 뒤집힌 pose는 base `TAG_LOST`로
처리한다. 정상적인 정면 관측에서는 tag `+Z`가 태그에서 base origin 쪽을 향하므로 이
내적은 음수다.

2026-09-03 Leader 실측에서는 tag 위치 약 `(0.312, 0.056) m`, tag `+Z`의 base XY
projection 약 `(-0.970, -0.244)`로 내적이 약 `-0.316`이었다. 과거 구현은 이 부호를
반대로 검사해 검출과 TF가 정상이어도 `TAG_LOST`를 출력했다. 현재 계산과 회귀 테스트는
이 실측 convention을 기준으로 한다.

현재 camera/base TF calibration은 실차 검증 완료 상태이므로 이 알고리즘 검증 중에는
URDF camera transform을 변경하지 않는다.

## 2. 상태와 예상 command

기존 consumer 호환성을 위해 기존 state 값을 단계 의미로 재사용하고
`FINAL_APPROACH`만 추가했다.

| State | 논리 단계 | 예상 raw command |
|---|---|---|
| `TAG_LOST` | pose/TF/normal 없음 또는 invalid | zero |
| `TURN_LEFT/RIGHT` | pre-align point를 향해 회전 | `linear.x=0`, signed `angular.z` |
| `APPROACH` | pre-align point 접근 | positive `linear.x`, bearing correction |
| `FINE_ALIGN_LEFT/RIGHT` | tag normal 기준 final yaw 정렬 | `linear.x=0`, signed `angular.z` |
| `FINAL_APPROACH` | 0.30 m에서 0.20 m로 저속 접근 | 최대 `0.02 m/s`, 작은 correction |
| `TOO_CLOSE` | final target overshoot, 후진 금지 | zero |
| `STABILIZING` | position `≤0.015 m`, yaw `≤4°` | zero |
| `ALIGNED` | 위 조건을 0.8초 연속 유지 | zero |

`STABILIZING` 중 한 조건이라도 깨지면 timer를 reset하고 해당 정렬 상태로 복귀한다.

## 3. 파라미터

Perception/state 설정은 `rescue_robot_apriltag/config/approach.yaml`에 있다.

| Parameter | 기본값 | 의미 |
|---|---:|---|
| `pre_align_distance` | `0.30 m` | 태그 면에서 pre-align base pose까지 거리 |
| `final_target_distance` | `0.20 m` | 태그 면에서 최종 base pose까지 거리 |
| `pre_align_position_tolerance` | `0.02 m` | pre-align 진입 반경 |
| `pre_align_heading_tolerance_deg` | `5°` | pre-align 접근 시작 heading 범위 |
| `final_position_tolerance` | `0.015 m` | 최종 2D position tolerance |
| `final_yaw_tolerance_deg` | `4°` | 최종 tag-facing yaw tolerance |
| `base_stable_time` | `0.8 s` | 연속 안정화 시간 |

Controller 설정의 `max_final_linear_speed=0.02 m/s`,
`max_final_angular_speed=0.08 rad/s`가 마지막 접근을 기존 raw limit보다 낮게 제한한다.
0.20 m는 초기 시험값이며 실제 gripper 파지 pose에서 측정한 base-link-to-tag 거리로
나중에 조정한다.

## 4. 기존 및 신규 topic

기존 `/leader/supply/base_relative_pose`, `base_forward_distance`,
`base_lateral_error`, `base_bearing`, `/leader/base_alignment/state`, raw/final cmd topic은
그대로 유지한다.

| Diagnostic topic | Type |
|---|---|
| `/leader/alignment/tag_normal_heading` | `std_msgs/msg/Float64` (outward `+Z` heading) |
| `/leader/alignment/prealign_target_pose` | `geometry_msgs/msg/PoseStamped` |
| `/leader/alignment/final_target_pose` | `geometry_msgs/msg/PoseStamped` |
| `/leader/alignment/control_target_pose` | `geometry_msgs/msg/PoseStamped` |
| `/leader/alignment/final_position_error` | `std_msgs/msg/Float64` |
| `/leader/alignment/final_yaw_error` | `std_msgs/msg/Float64` |

Pose topic은 모두 `base_link` frame과 원본 tag timestamp를 사용하므로 RViz Fixed Frame을
`base_link`로 설정해 표시할 수 있다. Heading/yaw 단위는 radian이다.
`tag_normal_heading`은 태그의 outward `+Z` 방향이며, final target pose의 robot yaw는
태그를 바라봐야 하므로 이 heading과 π rad 반대 방향이다.

## 5. 빌드와 자동 테스트

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_apriltag leader_approach_control rescue_robot_bringup
source install/local_setup.bash
colcon test --packages-select \
  rescue_robot_apriltag leader_approach_control stm32_bridge
colcon test-result --verbose
```

## 6. Phase 1: guard disabled 수동 pose 검증

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

통합 launch 시작 상태는 approach controller enabled, velocity guard disabled, motor
stopped다. 다른 터미널에서 확인한다.

```bash
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/alignment/tag_normal_heading
ros2 topic echo /leader/alignment/prealign_target_pose
ros2 topic echo /leader/alignment/final_target_pose
ros2 topic echo /leader/alignment/final_position_error
ros2 topic echo /leader/alignment/final_yaw_error
ros2 topic echo /leader/approach/cmd_vel_raw
ros2 topic echo /leader/cmd_vel
```

가장 중요한 수동 검증은 다음 두 경우다.

```text
Case 1                     Case 2
  TAG                        TAG
   ■                          ■
   │                           \
 ROBOT                         ROBOT
```

- Case 1: final position과 yaw가 맞으면 내부 `STABILIZING→ALIGNED` 판정이 가능해야 한다.
  현재 Leader 통합 launch의 post-align enabled 설정에서는 공개 `ALIGNED`가 추가
  odometry 직진과 정지 확인 후에 나온다. 아래 기존 visual-only 절차는
  `post_align_odom_enabled:=false`로 실행한다.
- Case 2: 카메라 중앙에서 태그를 바라보더라도 target position 또는 yaw error 때문에
  절대 ALIGNED가 되어서는 안 된다.
- Tag를 숨기면 즉시 `TAG_LOST`, raw/final command zero가 되어야 한다.

## 7. Phase 2: wheels-up 시험

바퀴를 지면에서 띄우고 주변을 정리한 후에만 guard를 enable한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

- `TURN_LEFT/RIGHT`: 제자리 좌/우 회전
- `APPROACH`: 전진하며 pre-align bearing 수정
- `FINE_ALIGN_LEFT/RIGHT`: 전진 없이 final yaw 회전
- `FINAL_APPROACH`: 더 낮은 속도로 전진
- `TAG_LOST`, `TOO_CLOSE`, `STABILIZING`, `ALIGNED`: 정지

## 8. Phase 3: 지면 저속 시험

태그 왼쪽, 오른쪽, 정면에서 각각 시작한다. 대표 sequence는 다음과 같다.

```text
TURN_LEFT/RIGHT → APPROACH → FINE_ALIGN_LEFT/RIGHT
→ FINAL_APPROACH → STABILIZING → ALIGNED
```

세 경우 모두 최종 robot `+X`가 태그 면을 향하고 태그 정면 normal line에 수렴해야 한다.
비정상적인 회전 또는 접근이 보이면 즉시 guard를 disable한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

## 9. Phase 4: 거리 확인

Pre-align에서 tag plane과 base origin 사이 normal 거리 약 0.30 m, ALIGNED에서 약
0.20 m, 마지막 접근 약 0.10 m인지 측정한다. 이는 camera optical z나 단순
`base_forward_distance`가 아니라 tag normal을 따른 거리다.

## 10. Safety와 known issue

- Controller/guard disabled, invalid pose/quaternion, NaN/inf, timeout, TAG_LOST는 zero다.
- Controller는 후진과 lateral velocity를 생성하지 않는다.
- Velocity guard의 clamp, reverse protection, timeout, startup-disabled 정책은 변경하지
  않았다.
- 먼 거리에서 `TURN_*↔TAG_LOST`가 반복되는 검출 불안정은 별도 known issue다. 이번
  작업은 debounce, detector tuning, 해상도, tag size를 변경하지 않는다.
- ALIGNED 이후 gripper grasp sequence 연결은 후속 작업이다.

## 11. 검출은 정상인데 base state가 TAG_LOST인 경우

먼저 detector와 TF를 분리해 확인한다.

```bash
ros2 topic echo /leader/apriltag/detections --once
ros2 topic echo /leader/supply/detected --once
ros2 run tf2_ros tf2_echo base_link 'leader/tag36h11:0'
ros2 topic echo /leader/supply/base_relative_pose --once
```

- detection이 있고 `supply/detected=true`인데 base pose만 없으면 base TF 또는 geometry
  validation 구간을 확인한다.
- 정상 정면 관측에서는 base XY로 projection한 tag `+Z`와 tag position의 내적이
  음수여야 한다.
- quaternion, XY projection 또는 내적 검증 실패 시 안전상 `TAG_LOST`를 출력한다.
- 이 경우 `apriltag_approach` 로그에 `tag-normal geometry is invalid` 경고와 구체적인
  검증 실패 원인이 5초 간격으로 출력된다.
- 이 진단을 위해 camera TF나 calibration offset을 변경하지 않는다.
