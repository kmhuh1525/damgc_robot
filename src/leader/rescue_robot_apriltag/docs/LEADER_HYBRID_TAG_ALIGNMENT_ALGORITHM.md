# Leader Hybrid AprilTag Alignment Algorithm

## 1. Overview

Leader는 전방 고정 RealSense D435로 AprilTag를 관측하고 차동구동으로 접근한다.
이 알고리즘은 먼 거리에서는 Tag 중심을 추적하고, 가까운 거리에서만 Tag 면의
orientation을 사용한다. 핵심 원칙은 다음과 같다.

> Far에서는 perception 유지가 우선이고, near에서는 orientation 정렬 정밀도를 높인다.

```text
AprilTag detection
        │
        ├─ position ── tag center bearing ────────┐
        │                                         │
        └─ orientation ── tag plane normal        │
                              │                   │
                       robot-facing sign          │
                              └──────────┬────────┘
                                         ▼
                          distance / visibility decision
                              │                   │
                           COARSE               NEAR
                        center tracking    bounded normal correction
                              └──────────┬────────┘
                                         ▼
                               FINAL_YAW_ALIGN
                                         ▼
                                FINAL_APPROACH
                                         ▼
                                  STABILIZING
                                         ▼
                                      ALIGNED
```

## 2. Project goal

목표는 단순히 카메라 중앙에서 Tag를 보는 것이 아니다. AprilTag가 부착된 물체의
실제 정면으로 Leader와 향후 gripper를 정렬하고, 파지 가능한 최종 pose를 만드는 것이다.
이번 범위는 `ALIGNED`까지이며 gripper 명령은 발행하지 않는다.

## 3. Previous center-only algorithm

기존 알고리즘은 `base_link`에서 Tag 위치 `(tag_x, tag_y)`를 얻고 다음 bearing으로
Tag 중심을 추적했다.

```text
tag_bearing = atan2(tag_y, tag_x)
```

이 방식은 실차에서 좌회전, 우회전, 전진과 Tag loss 정지가 안정적으로 검증됐다.
하지만 Tag 중심만 바라보면 다음처럼 Tag 면에 비스듬한 상태도 완료로 오인할 수 있다.

```text
       TAG
        ■
         \
          \
         ROBOT
```

Tag 위치만으로는 Tag가 어디에 있는지는 알 수 있지만 Tag 면이 어느 방향을 보는지는
알 수 없기 때문이다.

## 4. First orientation-direct algorithm

false alignment를 막기 위해 첫 orientation 기반 구현은 Tag normal에 0.30 m와 0.23 m
offset을 적용했다.

```text
             TAG
              ■
              │
         0.23 X  final target
              │
         0.30 O  pre-align target
```

로봇은 pre-align point를 향해 TURN/APPROACH하고, 그곳에서 Tag normal 기반 yaw를 맞춘
뒤 final target으로 접근했다.

## 5. Real hardware failure

첫 구현을 실차에서 시험했을 때 다음 현상이 확인됐다.

- 먼 거리부터 pre-align point를 향한 과도한 제자리 회전
- 실제 Tag가 카메라 화면 반대쪽으로 이동
- Tag가 D435 FOV를 벗어나 `TAG_LOST`
- orientation과 target pose noise에 따른 좌우 correction 반복
- `TURN ↔ APPROACH` 경계 진동과 지그재그
- pre-align normal line에 안정적으로 수렴하지 못함

기구, TF, 모터 경로 문제가 아니라 control target 선택 문제였다. Camera lateral TF는
이미 실차 보정되어 Tag/gripper 중심 정렬 시 lateral error가 약 `-0.0005 m`임이 확인됐다.
따라서 이 알고리즘은 camera TF를 변경하지 않는다.

## 6. Why direct pre-align targeting failed

pre-align point는 기하학적으로 올바르더라도 카메라가 계속 Tag를 볼 수 있다는 보장은
없다. 특히 전방 고정 카메라와 차동구동 로봇은 옆으로 이동할 수 없다.

```text
               TAG ■
                   │
                   O pre-align

ROBOT
```

로봇이 `O`를 정면에 놓으려고 크게 회전하면 Tag 자체는 화면 가장자리 방향으로 움직일
수 있다. Tag가 사라지는 순간 pose와 orientation feedback가 모두 끊기므로 정확한
geometric target보다 visibility가 먼저 보장되어야 한다.

## 7. Hybrid algorithm rationale

새 구조는 두 기존 방식의 장점을 결합한다.

- FAR: 실차 검증된 center tracking으로 visibility와 부드러운 접근을 보존한다.
- NEAR: 제한된 normal correction으로 Tag 정면선에 점진적으로 수렴한다.
- FINAL: orientation을 사용해 물체 면에 수직인 yaw와 최종 위치를 모두 검증한다.

Orientation을 완전히 제거하지 않는다. 제거하면 tilted object 지원과 side-looking false
alignment 방지가 다시 불가능해진다.

## 8. Position vs orientation

- Position은 “Tag가 어디에 있는가”를 나타낸다.
- Orientation은 “Tag 면이 어느 방향을 보고 있는가”를 나타낸다.

Position으로 center bearing과 거리를 계산하고, orientation으로 Tag 면의 normal을
계산한다. 두 정보는 역할이 다르며 hybrid controller는 단계에 따라 비중을 달리한다.

## 9. Tag normal concept

Tag frame의 local Z축은 Tag 평면에 수직이다. Quaternion 회전행렬의 세 번째 열,
즉 `R_base_tag × [0, 0, 1]`을 계산한 뒤 base XY 평면으로 projection한다.

```text
Tag plane       normal candidates
   / ■               ↙ n
  /                    ↗ -n
```

Camera optical-frame quaternion에서 단순 Euler yaw를 뽑지 않는다. Tilt가 있으면 optical
frame yaw는 ground-plane surface normal 방향과 동일하지 않기 때문이다.

## 10. Robot-facing normal selection

평면 normal에는 `+n`과 `-n` 두 후보가 있다. Detector의 +Z 부호를 front 방향으로
hard-code하지 않고 현재 로봇 위치로 선택한다.

```text
robot_direction = normalize((-tag_x, -tag_y))
dot = normal · robot_direction

dot < 0  → normal = -normal
dot > 0  → normal 유지
```

쉽게 말하면 Tag에서 robot으로 향하는 방향과 같은 쪽의 normal을 고른다. Dot가 거의
0이면 edge-on이라 부호를 안정적으로 선택할 수 없으므로 invalid sample로 처리한다.

선택은 normal median filter 전후에 수행한다. 이렇게 해야 반대 부호 sample이 filter
안에서 서로 상쇄되지 않는다.

## 11. Tilted object support

```text
                    TAG
                   / ■
                  /
ROBOT
```

World의 고정 축이 아니라 Tag pose에서 얻은 surface normal을 사용한다. Robot-facing
normal 위에 target을 만들고 최종 heading은 그 normal의 반대 방향, 즉 Tag를 바라보는
방향으로 둔다.

```text
                    TAG
                   / ■
                  /
               ROBOT
                 ↗
```

## 12. FAR / COARSE APPROACH

`tag_range > orientation_engage_distance`에서는 다음만 steering에 사용한다.

```text
tag_bearing = atan2(tag_y, tag_x)
```

- Tag 왼쪽: `TURN_LEFT`
- Tag 오른쪽: `TURN_RIGHT`
- 중앙: `APPROACH`

Pre-align/final geometry는 diagnostics용으로 계속 계산하지만 FAR control target은 실제
Tag center다. Orientation은 FAR steering을 지배하지 않는다.

## 13. Orientation engage logic

거리 mode에는 hysteresis가 있다.

```text
COARSE -- range <= 0.40 m --> NEAR
NEAR   -- range >  0.43 m --> COARSE
```

0.40~0.43 m 구간에서는 현재 mode를 유지하므로 한두 frame의 range noise가 mode를
반복 전환하지 않는다. Final phase 진입 후에는 distance noise로 COARSE로 돌아가지 않는다.

## 14. NEAR ALIGN

NEAR steering은 Tag center bearing을 anchor로 사용한다.

```text
normal_delta = wrap(prealign_bearing - tag_bearing)
limited_delta = clamp(normal_delta, ±6 deg)
steering_error = wrap(tag_bearing + limited_delta)
```

따라서 pre-align bearing 하나를 직접 따라가지 않는다. Normal geometry는 한 frame에
최대 6°의 제한된 bias만 추가하며, 여러 frame의 전진 과정에서 normal line으로 수렴한다.

## 15. FOV protection

NEAR와 final phase에서 Tag center bearing을 항상 감시한다.

- `|bearing| <= 11°`: 정상 NEAR correction
- `11° < |bearing| < 18°`: forward와 normal correction을 선형 감소
- `|bearing| >= 18°`: RECENTER 진입

Warning 영역에서 Tag가 가장자리에 가까워질수록 correction은 center tracking 쪽으로
돌아간다. Visibility를 잃으면 모든 visual control이 끊기므로 이 판단은 geometry보다
우선한다.

## 16. RECENTER mode

RECENTER에서는 다음을 강제한다.

```text
linear.x = 0
angular.z = center-tracking direction
normal correction = 0
```

18°에서 진입하고 11° 안으로 돌아올 때까지 유지한다. External state는 기존
`TURN_LEFT/RIGHT`를 재사용하고 `/leader/alignment/control_mode`가 `RECENTER`를 표시한다.

## 17. Hysteresis

한 threshold만 사용하면 noise가 다음을 만든다.

```text
4.9° → APPROACH
5.1° → TURN
4.8° → APPROACH
5.2° → TURN
```

새 기준은 다음과 같다.

- COARSE TURN 시작 8°, 종료 3°
- RECENTER 시작 18°, 종료 11°
- FINAL_APPROACH 진입 yaw 4°, re-align 8°
- NEAR 진입 0.40 m, 해제 0.43 m

## 18. Filtering

기존 `filter_window=5`를 재사용한다.

- Translation: x/y/z component median
- Normal: unit-vector x/y component median 후 재정규화
- 같은 timestamp는 중복 삽입하지 않음

Normal을 angle scalar가 아닌 vector로 filter하므로 `-pi/+pi` 경계에서 단순 angle 평균
문제가 없다. 중복 low-pass filter는 추가하지 않았다.

## 19. Pre-align pose

Robot-facing normal을 `n`이라 하면:

```text
prealign = tag_position + 0.30 × n
```

0.30 m 값은 유지한다. FAR target이 아니라 NEAR geometry reference와 final yaw를 시작할
위치다. Pre-align을 조금 지나쳤지만 final target이 아직 앞이면 후진하지 않고 final
phase로 넘어간다.

## 20. FINAL_YAW_ALIGN

Pre-align 영역에 들어오면 robot linear velocity를 zero로 두고 다음 heading을 맞춘다.

```text
target_yaw = atan2(-normal_y, -normal_x)
final_yaw_error = wrap(target_yaw)  # base heading is zero in current base frame
```

External state는 `FINE_ALIGN_LEFT/RIGHT`다. 최대 angular speed는 `0.08 rad/s`로 제한된다.

## 21. FINAL_APPROACH

Yaw가 5° 이내이면 0.30 m에서 0.23 m target으로 저속 접근한다.

- 최대 linear: `0.02 m/s`
- 최대 angular: `0.08 rad/s`
- 작은 yaw correction과 final target lateral correction 허용
- yaw가 8°를 넘으면 forward를 중단하고 FINAL_YAW_ALIGN 복귀

큰 오차를 가진 채 곡선으로 마지막 접근을 계속하지 않는다.

## 22. STABILIZING / ALIGNED

다음 두 조건을 모두 만족해야 한다.

```text
final_position_error <= 0.020 m
abs(final_yaw_error) <= 5 deg
```

첫 valid frame은 `STABILIZING`이다. 두 조건이 `base_stable_time=0.30 s` 동안 연속으로
유지된 뒤 서로 다른 source timestamp의 valid observation 3개를 추가로 확인해야
`ALIGNED`가 된다. 하나라도 벗어나면 stability와 confirmation을 reset한다. 동일 TF
timestamp는 timer가 반복 처리해도 중복 count하지 않는다. Tag 중심만 보는 side-looking
pose는 position 또는 yaw error가 남으므로 ALIGNED가 될 수 없다.

`ALIGNED`는 정렬 상태머신의 내부 latch다. `post_align_odom_enabled=false`이면
기존처럼 바로 public `ALIGNED`/zero가 된다. Enabled이면 이 decision을
post-align 시작 신호로만 쓰며 odometry 진행과 정지 확인 후에 public `ALIGNED`를 낸다.
`reset()`, selected Tag ID 변경, node 재시작은 내부 latch를 해제한다.

`STABILIZING`에서 직전 valid pose가 두 tolerance를 만족한 경우에만 최대 `0.20 s`의
Tag-loss grace를 허용한다. grace 동안 state는 `STABILIZING`, command는 zero이고
confirmation은 증가하지 않으며, Tag 없는 시간은 stability elapsed에서 pause한다.
timeout은 기존 `TAG_LOST`/safe stop으로 전환한다. 다른 phase의 loss에는 grace를
적용하지 않는다.

## 23. Full state/mode transition

```text
                 ┌──────────────────────────────┐
                 │                              │
TAG_LOST ─ valid ▼                              │ tag lost/invalid
             COARSE_TRACK                       │
       TURN_LEFT/RIGHT ↔ APPROACH                │
                 │ range <= 0.40                 │
                 ▼                              │
              NEAR_ALIGN ── FOV >= 18 ─► RECENTER
                 ▲                    FOV <= 11 │
                 └──────────────────────────────┘
                 │ pre-align reached
                 ▼
          FINAL_YAW_ALIGN
                 │ yaw <= 5
                 ▼
           FINAL_APPROACH
                 │ yaw > 8
                 └────────────► FINAL_YAW_ALIGN
                 │ position <= 0.020 and yaw <= 5
                 ▼
            STABILIZING
                 │ continuous 0.30 s + 3 fresh valid samples
                 ▼
              ALIGNED

Any phase + final target behind tolerance → TOO_CLOSE → zero
Ineligible/far phase + lost/invalid/stale → TAG_LOST → zero
Eligible final close-range loss          → blind handoff evaluation
```

## 24. Parameters table

| Parameter | Default | Unit | Meaning | Tuning direction |
|---|---:|---|---|---|
| `orientation_engage_distance` | 0.40 | m | NEAR 진입 거리 | 늘리면 normal 정렬 구간 증가 |
| `orientation_disengage_distance` | 0.43 | m | COARSE 복귀 거리 | engage보다 커야 함 |
| `turn_enter_error_deg` | 8.0 | deg | COARSE 제자리 회전 시작 | 줄이면 더 자주 TURN |
| `turn_exit_error_deg` | 3.0 | deg | COARSE TURN 종료 | 줄이면 더 정확히 중앙화 |
| `tag_recenter_enter_deg` | 18.0 | deg | RECENTER 시작 | 줄이면 FOV 보호 강화 |
| `tag_recenter_exit_deg` | 11.0 | deg | RECENTER 종료 | 줄이면 더 중앙까지 회전 |
| `near_normal_correction_limit_deg` | 6.0 | deg | NEAR normal bias 최대값 | 늘리면 수렴 빠름/FOV 위험 증가 |
| `pre_align_distance` | 0.30 | m | Tag 면부터 pre-align 거리 | grasp geometry 확인 전 유지 |
| `final_target_distance` | 0.23 | m | 최종 base 거리 | 향후 gripper 실측 후 조정 |
| `pre_align_position_tolerance` | 0.02 | m | final phase 진입 반경 | 늘리면 final yaw를 일찍 시작 |
| `final_position_tolerance` | 0.020 | m | ALIGNED 위치 오차; current visual-only test value | 늘리면 완료 판정 완화 |
| `final_yaw_tolerance_deg` | 5.0 | deg | final yaw 및 진입 허용; current visual-only test value | 늘리면 정렬 정확도 완화 |
| `final_realign_yaw_error_deg` | 8.0 | deg | final 접근 중 재정렬 | 줄이면 더 자주 정지/재정렬 |
| `base_stable_time` | 0.30 | s | confirmation 전 안정 조건 유지 시간 | 늘리면 완료 확정 강화 |
| `aligned_confirm_samples` | 3 | sample | stability 후 fresh valid confirmation 수 | 늘리면 완료 확정 강화 |
| `stabilizing_tag_loss_grace_sec` | 0.20 | s | STABILIZING 전용 loss grace | 늘리면 loss 복귀 허용 증가 |
| `filter_window` | 5 | sample | translation/normal median | 늘리면 안정적이나 지연 증가 |
| `near_max_angular_speed` | 0.10 | rad/s | NEAR/RECENTER 회전 제한 | 줄이면 부드럽지만 느림 |
| `max_final_linear_speed` | 0.02 | m/s | final 접근 최대 속도 | 낮추면 final 안정성 증가 |
| `max_final_angular_speed` | 0.08 | rad/s | final yaw/correction 제한 | 낮추면 overshoot 감소 |

### 24.1 Current Visual-Only Validation Tuning

실차에서 hybrid alignment algorithm이 이전보다 안정적으로 동작하는 상태에서,
close-range odometry blind fallback을 일시적으로 비활성화하고 순수 visual final
alignment를 먼저 검증한다. Tag 높이를 조정해 약 `0.23 m`의 final target까지
AprilTag가 계속 camera frame에 보이도록 한 조건이다. Blind 구현을 삭제한 것이
아니라 parameter로만 disabled했으므로, 이후 `blind_final_approach_enabled`를
`true`로 되돌리면 기존 fallback을 재사용할 수 있다.

| Parameter | Previous validation value | Current test value | Meaning |
|---|---:|---:|---|
| `blind_final_approach_enabled` | `true` | `false` | visual-only 검증; close-range loss는 기존 stop behavior |
| `final_position_tolerance` | `0.015 m` | `0.020 m` | final target planar position error 허용 범위 |
| `final_yaw_tolerance_deg` | `4.0 deg` | `5.0 deg` | final target yaw error 허용 범위 |
| `base_stable_time` | `0.8 s` | `0.30 s` | confirmation 전 두 조건 유지 시간 |
| `aligned_confirm_samples` | - | `3` | fresh valid confirmation sample 수 |
| `stabilizing_tag_loss_grace_sec` | - | `0.20 s` | STABILIZING short dropout grace |

Position 2 cm와 yaw 5°는 AprilTag pose noise와 바닥 주행 오차를 고려할 때 기존
1.5 cm/4°가 초기 실차 validation에서 다소 엄격할 가능성을 소폭 완화한 값이다.
정확도를 과도하게 희생하지 않고 향후 gripper 파지 여유도 남기기 위해 이 정도로만
조정한다. Position과 yaw 조건을 모두 만족한 상태가 `0.30 s` 연속 유지되고 fresh
valid sample 3개가 확인돼야 visual `ALIGNED`가 된다. 이 설정은 실차 tuning 단계의
검증 설정이다.

## 25. Safety behavior

다음 조건은 raw command zero다.

- `TAG_LOST`, invalid pose, invalid quaternion
- normal XY projection 또는 robot-facing sign이 ambiguous
- NaN/inf, stale/future timestamp
- pose/mode/state가 coherent하지 않음

## 31. Atomic command boundary

기존에는 control target pose, `control_mode`, `base_alignment/state`가 서로 다른
ROS topic으로 전달되어 callback 지연 시 서로 다른 frame이 결합될 가능성이 있었다.
현재 controller 입력은 `leader_alignment_msgs/msg/LeaderAlignmentCommand` 하나로
통합된다. 이 메시지는 하나의 `header`(timestamp/frame), `target_pose`,
`control_mode`, `alignment_state`를 포함하며 perception의 동일한 state-machine
evaluation 결과에서 한 번 생성·publish된다.

`/leader/alignment/command`는 제어용 authoritative input이고, 기존 문자열 topic과
`control_target_pose`는 관찰/검증용 diagnostic output이다. Controller는 diagnostic
topic을 구독해 coherence를 추정하지 않으며, command header timestamp만으로 기존
freshness watchdog을 수행한다. stale, invalid, TAG_LOST command는 zero Twist로
fail-closed 된다.
- controller disabled
- `TOO_CLOSE`, `STABILIZING`, `ALIGNED`

Guard disabled이면 raw가 non-zero여도 `/leader/cmd_vel`은 zero다. Velocity guard의 clamp,
reverse protection, timeout, slew와 startup-disabled 정책은 수정하지 않았다.

## 26. Comparison

| Item | Center-only | Orientation-direct | Hybrid |
|---|---|---|---|
| Tag visibility | 높음 | 낮음 | 높음, FOV 보호 |
| Orientation 사용 | 없음 | 전 구간 | NEAR/final |
| Tilted Tag | 불가 | 가능 | 가능 |
| False ALIGNED 방지 | 약함 | 가능 | 가능 |
| FOV 안정성 | 좋음 | 실차 실패 | center anchor + RECENTER |
| Oscillation 위험 | 낮음 | 높음 | hysteresis/clamp |
| 실차 tuning | 쉬움 | 어려움 | 단계별 parameter |
| Gripper 정면 접근 | 부적합 | 기하학상 적합 | visibility 포함 적합 |

## 27. Expected trajectory examples

```text
FAR left start                   NEAR convergence

       TAG ■                         TAG ■
                                          │ normal line
ROBOT ↖                            ROBOT ↗ │

center tracking first             bounded correction over time
```

Tag가 18° 밖으로 밀리면 trajectory를 계속 강제하지 않고 제자리 recenter 후 재평가한다.

## 28. Tilted Tag example

```text
1. FAR                              2. NEAR/FINAL

                    TAG                                TAG
                   / ■                                / ■
                  /                                  /
ROBOT ── center tracking                         ROBOT
                                                   ↗
```

FAR에서 Tag 중심을 보존하고, NEAR에서 robot-facing normal을 고른 뒤 그 line으로 수렴한다.
마지막에는 `-normal` 방향으로 heading을 맞추고 0.23 m target까지 접근한다.

## 29. Known limitations

- 먼 거리에서 작은 AprilTag 검출 자체가 불안정한 문제는 별도 perception 문제다.
- 이번 작업은 TAG_LOST debounce를 추가하지 않는다.
- 매우 edge-on인 Tag는 projected normal 또는 sign이 ambiguous해 zero가 될 수 있다.
- 0.40 m engage와 6° correction은 초기 실차 tuning 값이다.
- 필요 시 larger tag, camera resolution, detector tuning, 제한적 dropout 처리를 별도
  작업으로 검토한다.

## 30. Future Gripper integration

향후 연결 지점은 `ALIGNED`다. Gripper sequence는 `ALIGNED`가 final position과 yaw를
안정적으로 만족한 뒤에만 시작해야 한다. 현재 알고리즘의 최종 목적은 Tag 부착 물체의
정면으로 robot/gripper를 정렬해 파지 가능한 pose를 제공하는 것이다.

## 31. Close-Range Tag Loss and Odometry Blind Final Approach

### 31.1 문제 배경

Final approach에서 로봇이 Tag 면에 너무 가까워지면 AprilTag 사각형 전체가 D435
image frame 안에 들어오지 않을 수 있다. 이 경우 정렬이 잘못되어서가 아니라 Tag의
일부가 잘려 detector가 유효한 pose를 만들지 못해 `TAG_LOST`가 발생한다.

### 31.2 제한적인 sensor handoff

기존 FAR center tracking, NEAR normal alignment, robot-facing normal 선택, FOV recenter,
hysteresis, final yaw와 visual final approach는 변경하지 않는다. 마지막 final 몇 cm에서
발생하는 close-range loss만 다음의 제한된 handoff로 처리한다.

```text
FINAL_APPROACH
      │
  new Tag samples continue?
    /               \
  YES                NO
   │                  │
 visual final       source-stamp freshness timeout
 approach             │
                    eligibility
                    /        \
                  NO          YES
                  │             │
                 STOP       BLIND_FINAL_APPROACH
                                │
                             odometry
                                │
                         target reached
                                │
                      ALIGNED + zero latched
```

TF lookup 성공은 새로운 AprilTag detection을 의미하지 않는다. `lookup_transform(...,
Time())`은 detector가 publish를 멈춘 뒤에도 TF buffer가 보관한 마지막 transform을
반환할 수 있다. 따라서 timer 실행 시각이 아니라 transform `header.stamp`를 source
timestamp로 사용한다. 같은 tag의 source stamp가 이전에 처리한 stamp보다 클 때만 새
visual observation으로 인정하며, 같은 stamp를 반복해서 읽어도 last-valid snapshot과
freshness를 갱신하지 않는다.

Generic `TAG_LOST`는 여전히 즉시 zero command다. 다만 FINAL_APPROACH에서 실제 source
stamp가 `blind_last_tag_max_age` 동안 갱신되지 않은 경우에는 global `tag_timeout`을
기다리지 않고 close-range loss 후보를 평가한다. 오직 직전 valid phase가 실제
`FINAL_APPROACH`였고, close range·fresh pose·작은 yaw/cross-track 오차·짧은 remaining
distance·valid odometry를 모두 만족하는 경우에만 internal mode
`BLIND_FINAL_APPROACH`를 사용한다. `FINE_ALIGN_LEFT/RIGHT`, TURN, COARSE, NEAR,
RECENTER 중 loss는 항상 stop한다.

### 31.2.1 Sample-loss timeout과 visual handoff age

두 freshness 값은 서로 다른 목적을 가진다. `blind_last_tag_max_age`는 새 source Tag
sample이 없다고 판단하는 loss detection timeout이고, `blind_handoff_max_age`는 loss를
감지한 뒤 마지막 valid visual snapshot을 odometry handoff에 사용할 수 있는 최대 age다.
기존처럼 하나의 `0.25 s` 값을 두 판단에 함께 사용하면 `0.25 s` 이후 loss를 판단하는
순간 같은 snapshot이 이미 handoff 불가가 되는 논리 충돌이 발생한다. 현재 기본값은
다음과 같다.

```text
sample-loss timeout = blind_last_tag_max_age = 0.25 s
visual handoff age  = blind_handoff_max_age  = 0.40 s
```

따라서 실제 callback이 마지막 sample 뒤 `0.30 s`에 실행되어도 loss candidate가 되고,
visual age `0.30 s <= 0.40 s`이므로 나머지 strict eligibility를 평가할 수 있다.
`blind_handoff_max_age`는 `blind_last_tag_max_age`보다 커야 하며, 너무 오래된 visual
snapshot을 blind 주행에 사용하지 않도록 `0.40 s` 상한을 유지한다.

### 31.3 거리와 odometry

`last_valid_tag_x`는 camera optical `z`가 아니라 `base_link`의 +X 전방 거리다.
마지막 valid final-approach sample에서 다음 식으로 blind 계획을 한 번 계산한다.

```text
planned_blind_distance = last_valid_tag_x - final_target_distance
```

예를 들어 `0.265 m - 0.230 m = 0.035 m`이다. 음수면 reverse하지 않으며, 작은 음수는
zero로 취급하고, 큰 음수 또는 `blind_max_distance` 초과는 blind를 거부한다.

Blind 시작 순간 `odom` frame의 `x`, `y`, `yaw`를 한 번 snapshot한다. 진행거리는 odom
X 차이가 아니라 시작 heading 방향으로 displacement를 projection한다.

```text
dx = current_x - start_x
dy = current_y - start_y
forward_progress = cos(start_yaw) * dx + sin(start_yaw) * dy
```

이 방식은 odom/world X축과 로봇의 시작 전진 방향이 달라도 실제 전진량을 측정한다.
Blind 중에는 저속 positive `linear.x`만 사용하고 `angular.z`는 0이다. 진행량이 계획량에
도달하면 post-align disabled에서는 기존대로 zero 및 `ALIGNED`가 된다. Enabled에서는
별도 post-align odometry 이동을 시작하고 그 완료·정지 확인 후 public `ALIGNED`가 된다.
Latch가 유지되는 동안 Tag loss나 cached TF는 `TAG_LOST`로 되돌리거나 같은 blind plan을
다시 만들지 못한다.

### 31.4 Re-acquisition과 safety

Blind 중 valid Tag가 재검출되면 기존 visual 정보를 우선한다. blind snapshot과 plan을
폐기하고 현재 pose를 기존 hybrid state machine에 입력한다. Invalid/stale pose는 재검출로
인정하지 않으며, 동일한 TF stamp도 재검출로 인정하지 않는다. 반대로 blind 완료 후의
Tag 재검출은 terminal `ALIGNED`를 해제하지 않는다. 현재 Leader node에는 perception
reset service가 없으므로 새 approach cycle은 `apriltag_approach` 프로세스 재시작으로
시작하며, 그때 completion latch가 초기화된다.

Odom이 stale/unavailable/invalid이거나 NaN/inf, 비정상 jump, 음수 progress 또는 watchdog
timeout이 발생하면 즉시 zero하고 `ALIGNED`로 전환하지 않는다. 시간은 주행거리의 대체가
아니며 `blind_max_duration`은 stuck/odom failure용 watchdog일 뿐이다. Blind는 forward-only이고
reverse command를 만들지 않는다.

### 31.5 ALIGNED 의미

일반 visual `ALIGNED`는 AprilTag pose로 최종 위치와 yaw를 확인한 내부 상태다.
Blind-final `ALIGNED`는 마지막 valid fine-aligned visual pose와 짧은 odometry translation으로
추정한 상태다. Public state는 기존 호환성을 위해 `ALIGNED`로 유지하지만
post-align 실행 여부는 별도 `post_align_*` diagnostic으로 구분한다.

### 31.6 Parameters and diagnostics

| Parameter | Default | Unit | Meaning |
|---|---:|---|---|
| `blind_final_approach_enabled` | `true` | bool | fallback enable; current visual-only test value is `false` |
| `blind_activation_max_tag_x` | `0.30` | m | blind 허용 최대 base +X |
| `blind_max_distance` | `0.12` | m | 계획 가능한 최대 remaining distance |
| `blind_last_tag_max_age` | `0.25` | s | 새 sample loss detection timeout |
| `blind_handoff_max_age` | `0.40` | s | visual-to-odometry handoff 최대 age |
| `blind_max_duration` | `5.0` | s | blind watchdog |
| `odom_topic` | `/leader/odom/raw` | topic | 기존 Leader wheel odometry |
| `blind_final_speed` | `0.015` | m/s | blind forward speed; final speed 이하 |

기존 `final_yaw_tolerance_deg`와 `final_position_tolerance`를 각각 yaw와 lateral/cross-track
gate에 재사용한다. 다음 diagnostic으로 blind 상태를 확인한다.

```text
/leader/alignment/blind_final_approach_active
/leader/alignment/last_valid_tag_x
/leader/alignment/blind_planned_distance
/leader/alignment/odom_forward_progress
```

설계 이유는 전체 hybrid controller를 다시 바꾸는 것이 아니라, 카메라가 마지막 몇 cm를
관측하지 못하는 한정된 상황에서만 vision에서 기존 odometry로 sensor handoff하기 위해서다.
이 fallback은 vision-confirmed final pose를 대체하지 않는다. Post-align enabled이면
visual 또는 blind 어느 경로든 mandatory odometry advance를 거친 뒤에만
후속 gripper sequence의 연결점인 public `ALIGNED`를 발행한다.
