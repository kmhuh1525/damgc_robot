# follower_supply_perception

- 파일 경로: `README.md`

Jetson Orin Nano의 ROS 2 Humble 환경에서 AprilTag 상대 위치를 사용해 공급 대상의
접근·정렬 상태를 판단하기 위한 Python 패키지입니다.

프로젝트 계획상 이 패키지는 팔로워의 “AprilTag 기반 상대 위치 보정”과
물품 정밀 접근 입력을 담당합니다. 전체 목표와 현재 통합 순서는
[`docs/Plan.md`](../../../docs/Plan.md)와
[`docs/STATUS_AND_ROADMAP.md`](../../../docs/STATUS_AND_ROADMAP.md)를 따릅니다.
이 패키지의 상태 출력만으로 계획의 정밀 접근·주행·파지가 완료된 것은 아닙니다.

현재 단계에서는 기존 camera-frame 상태를 유지하면서 TF2 exact-stamp 변환,
`base_link` pose·metric·상태, 별도 approach controller, deterministic command selector,
최종 Follower safety guard까지 software pipeline을 구성했습니다.
확정된 요구사항은
[`docs/TASK_SPEC_APRILTAG_APPROACH.md`](docs/TASK_SPEC_APRILTAG_APPROACH.md)에
기록되어 있습니다.

단위 테스트 결과와 하드웨어 미검증 범위는
[`docs/TEST_RESULTS_APRILTAG_APPROACH.md`](docs/TEST_RESULTS_APRILTAG_APPROACH.md)에
기록되어 있습니다.

실제 ROS 그래프 통합 확인 결과는
[`docs/INTEGRATION_CHECK_APRILTAG_APPROACH.md`](docs/INTEGRATION_CHECK_APRILTAG_APPROACH.md),
물리 태그 수동 시험은
[`docs/MANUAL_STATE_TEST.md`](docs/MANUAL_STATE_TEST.md)를 참고합니다.

launch 구성과 검증 결과는
[`docs/LAUNCH_VALIDATION_APRILTAG.md`](docs/LAUNCH_VALIDATION_APRILTAG.md)에
기록되어 있습니다.

종합 설계·운영 가이드는
[`docs/APRILTAG_APPROACH_NODE_GUIDE.md`](docs/APRILTAG_APPROACH_NODE_GUIDE.md),
전체 구현 변경 기록은
[`docs/IMPLEMENTATION_RECORD.md`](docs/IMPLEMENTATION_RECORD.md)를 참고합니다.

현재 base-link velocity pipeline의 설계, 전체 파라미터, 선택 빌드·자동시험 결과와
실카메라 검증 절차는
[`docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md`](docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
기준으로 합니다.

최신 Leader hybrid alignment의 Follower 이식 구조, atomic command, USB timestamp
처리, blind-final safety gate와 단계별 검증 절차는
[`docs/FOLLOWER_HYBRID_ALIGNMENT_MIGRATION.md`](docs/FOLLOWER_HYBRID_ALIGNMENT_MIGRATION.md)를
기준으로 합니다. Blind final은 코드와 자동시험만 포함하며 기본값은 반드시
`false`입니다.

현재 hybrid base 정렬의 안정화 계약은 다음과 같습니다.

- `base_stable_time=0.30 s` 이후에도 서로 다른 source stamp를 가진 fresh observation
  3개가 확인되어야 `ALIGNED`로 latch됩니다.
- `FINAL_APPROACH`와 `STABILIZING`에서 태그가 잠시 사라지면 각각 `0.30 s` 동안 기존
  state/control mode만 유지합니다. 이때 stale target pose를 재사용하지 않으며 raw
  `cmd_vel`은 반드시 zero입니다.
- grace 안에 strictly newer observation이 들어오면 visual control로 복구하고, grace가
  끝날 때까지 들어오지 않으면 blind가 기본값처럼 disabled인 경우 `TAG_LOST`로 갑니다.
- `tag_timeout=2.0 s`는 source stamp sanity bound, `tag_receipt_timeout=0.35 s`는 local
  monotonic dropout 기준입니다. Duplicate TF는 새 observation이나 grace reset으로
  인정하지 않습니다.
- Controller enable/disable이 `/follower/approach/enabled`로 perception에 전달되며, 새
  approach session 시작 시 이전 `ALIGNED` latch와 안정화/grace 이력을 초기화합니다.

실제 바퀴를 사용하는 시험은
[`docs/FOLLOWER_WHEEL_DRIVE_TERMINAL_TEST.md`](docs/FOLLOWER_WHEEL_DRIVE_TERMINAL_TEST.md)의
터미널별 절차를 따릅니다. Receive-only I2C, stand 위 저속 pulse, 지상 odometry,
AprilTag 자동 접근 순서로 진행하며 `/follower/safe_cmd_vel` 직접 publish는 금지합니다.

노드 실행 진입점은 `apriltag_approach_node`입니다. 출력 토픽은 상대 이름을 사용하므로
요구된 `/follower/...` 이름으로 사용하려면 노드를 `follower` namespace에서 실행해야
합니다.

`config/approach.yaml`의 모든 수치는 초기 기능 시험용입니다. 특히
`target_distance`는 실제 그리퍼 동작 거리로 확정된 값이 아니며 현장 측정 후 조정해야
합니다.

## 환경 확인

- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10
- 작업공간: `~/damgc_robot`
- 패키지: `~/damgc_robot/src/follower/follower_supply_perception`

향후 빌드와 테스트를 실행할 때는 각 셸에서 ROS 환경을 먼저 설정합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/damgc_robot
colcon build --packages-select follower_supply_perception
source install/local_setup.bash
colcon test --packages-select follower_supply_perception
colcon test-result --verbose
```

이 패키지 자체의 node는 perception과 상태 판단만 담당합니다. raw command, command
ownership, final safety는 각각 `follower_approach_control`,
`follower_command_selector`, `follower_control`이 담당합니다. 통합 launch는 이 패키지들을
`stm32_bridge`까지 연결하지만 STM32 firmware와 motor control 구현은 변경하지 않습니다.

## Launch 실행

기존 수동 카메라·Rectify·AprilTag 노드를 먼저 해당 터미널에서 종료한 다음 전체
주행 software pipeline을 실행합니다.

현재 Follower STM32 I2C slave firmware가 준비되지 않은 상태에서는 bridge만 제외한
software-only 모드를 사용합니다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  use_stm32_bridge:=false
```

STM32 I2C firmware가 준비된 실제 robot에서는 기본 실행을 사용합니다. 기본 bridge
설정은 `follower` namespace, `i2c`, `/dev/i2c-7`, address `66 (0x42)`, I2C write enabled다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py
```

두 모드 모두 approach controller는 enabled, selector는 `APPROACH`, velocity guard는
disabled 상태로 시작합니다. 따라서 controller 내부 command가 생성되더라도 실제 motor
command를 통과시키려면 사용자가 guard를 명시적으로 열어야 합니다.

### 통합 Dynamixel gripper

Follower gripper는 별도 launch를 새로 실행하는 구성이 아니라 기존
`follower_apriltag_drive.launch.py`에 이미 포함되어 있다. 현재 안전 설정은 기존 통합을
유지하면서 Dynamixel startup pose/torque와 Tag-loss idle pose를 Follower에서
비활성화하고, 검증된 RX-64 저속 lift를 기본 활성화한다.

```text
Follower Camera
      ↓
AprilTag Detection
      ↓
/follower/supply/detected ───────→ Gripper Sequence
                                      │
                                      └─ Tag detected
                                         → RX-28 OPEN 950

Hybrid Alignment
      ↓
/follower/base_alignment/state ──→ Gripper Sequence
                                      │
                                      └─ ALIGNED
                                         → RX-28 CLOSE 350
                                         → lift_enabled?
                                             false → DONE
                                             true  → close_wait
                                                   → RX-64 speed 50 / raw 20
                                                   → LIFTING → DONE

Dynamixel profile/namespace: follower
Default: lift_enabled=true, lift_raw=20, rx64_speed=50
Follower RX-64 range: raw 20..270 (20=highest, 270=lowest)
```

기본값은 다음과 같다.

| 설정 | 기본값 | 의미 |
|---|---:|---|
| `gripper_enabled` | `true` | Dynamixel node와 sequence의 master gate |
| `robot` | `follower` | Follower ID/range/profile 선택 |
| `gripper_port` | `/dev/ttyUSB0` | Follower U2D2 device |
| child `baudrate` | `115200` | Dynamixel bus baudrate |
| `gripper_open_raw` | `950` | RX-28 OPEN Goal Position |
| `gripper_close_raw` | `350` | RX-28 CLOSE Goal Position |
| `lift_enabled` | `true` | RX-64 정상 lift path의 enable |
| `lift_raw` | `20` | RX-64 Goal Position; valid range 20..270 (20=highest, 270=lowest) |
| `rx64_speed` | `50` | RX-64 Moving Speed raw 값 |
| child `close_wait` | `3.0 s` | lift enabled일 때 CLOSE와 LIFT 사이 대기 |
| child `startup_pose_enabled` | shared `true`, Follower override `false` | launch 직후 Goal Position write 차단 |
| child `startup_torque` | shared `true`, Follower override `false` | launch 직후 torque write 차단 |
| child `tag_lost_idle_enabled` | shared `true`, Follower override `false` | Tag flicker에 의한 idle reposition 차단 |

기본 sequence는 `Tag detected → RX-28 OPEN 950 → approach → ALIGNED → RX-28
CLOSE 350 → close_wait 3.0 s → RX-64 speed 50 / Goal Position 20 → DONE`이다.
Follower RX-64 운용 범위는 지정된 raw `20..270`이며, `20=최고`, `270=최저`다. 기존
raw `500 → 300` 시험 기록은 이전 프로파일 기준이므로 현재 follower 목표로 사용하지 않는다.
속도를 낮춘 목적은 파지 후 lift를 천천히 동작시켜 물체 이동을 안정화하는 것이다.
`lift_raw`는 Goal Position raw target이고 `rx64_speed`는 주소 32의 Moving Speed raw
값이다. Speed raw `0`은 정지가 아니라 속도 제한 없는 최대 속도를 의미한다.

OPEN과 CLOSE는 각각 다음 targeted raw command를 사용한다.

```text
OPEN  [-1, 950, -1, 1]
CLOSE [-1, 350, -1, 1]
       [rx64_raw, rx28_raw, rx64_torque, rx28_torque]
```

`-1`은 해당 position 또는 torque를 변경하지 않는 sentinel이다. 따라서 두 명령은
RX-64 position/torque를 건드리지 않고 RX-28 torque만 켠 뒤 목표 위치를 쓴다. RX-64
Moving Speed는 연결 시 한 번 설정되지만 Goal Position이나 torque를 쓰지 않으므로 motion을
발생시키지 않는다. Tag 검출 전 position/torque write는 없고 temporary Tag loss도 어느
모터의 idle reposition을 유발하지 않는다. RX-28 Moving Speed에는 write하지 않으므로
기존 RX-28 OPEN/CLOSE 속도 및 torque/position 동작은 그대로다.

Gripper를 완전히 제외하려면 다음처럼 실행한다. `lift_enabled:=true`나 유효한
`lift_raw`가 함께 주어져도 `gripper_enabled=false`가 우선한다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  gripper_enabled:=false
```

자동 lift만 끄려면 다음처럼 실행한다. RX-28 OPEN/CLOSE 후 DONE으로 진행하며 RX-64
Goal Position은 발행되지 않는다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  lift_enabled:=false
```

속도나 lift target을 명시적으로 바꿔 시험하려면 다음처럼 실행한다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  rx64_speed:=50 lift_raw:=20
```

유효한 값이면 `ALIGNED → RX-28 CLOSE → close_wait → RX-64 lift_raw → LIFTING →
DONE`으로 진행한다. `lift_raw`가 finite integer가 아니거나 Follower RX-64 configured range
밖이면 RX-28 CLOSE를 유지하고 RX-64 command 없이 error status를 남긴 뒤 DONE으로
끝난다. Tag-loss idle policy와 정상 lift enable은 서로 독립적이다.

정확한 관찰 지점은 다음과 같다.

| 용도 | 이름 |
|---|---|
| detection | `/follower/supply/detected` |
| alignment | `/follower/base_alignment/state` |
| raw command | `/follower/dynamixel/command` |
| semantic command | `/follower/gripper/command` |
| Dynamixel status | `/follower/dynamixel/status` |
| sequence node/status | `/follower/gripper_sequence`, `/follower/sequence/status` |
| wheel enable service | `/follower/velocity_guard/enable` |

Leader shared behavior는 기존 기본값을 유지한다.

| 항목 | Leader | Follower integrated launch |
|---|---|---|
| profile | `leader` | `follower` |
| Dynamixel namespace | `/leader` | `/follower` |
| detection | `/leader/supply/detected` | `/follower/supply/detected` |
| alignment | `/leader/base_alignment/state` | `/follower/base_alignment/state` |
| RX-28 OPEN/CLOSE | `1000` / `450` | `950` / `350` |
| lift default | enabled, `300` | enabled, `300` |
| RX-64 speed | raw `50` | raw `50` |
| startup pose | `startup_pose_enabled=false` | `startup_pose_enabled=false` |
| startup torque | `startup_torque=false` | `startup_torque=false` |
| Tag-loss idle | `tag_lost_idle_enabled=false` | `tag_lost_idle_enabled=false` |

#### Hardware validation 순서

실제 검증은 사람과 물체를 작업 반경에서 치우고 물리적 비상 정지를 준비한 뒤 아래
순서를 지킨다. 자동시험에서는 wheel guard를 열거나 Dynamixel command를 발행하지 않는다.

1. `gripper_enabled:=false`로 기존 AprilTag/drive graph를 회귀 확인한다.
2. 기본 launch를 wheel guard가 닫힌 상태로 실행하고 Tag가 없을 때 RX-28, RX-64,
   wheel이 움직이지 않는지와 runtime `rx64_speed=50`을 확인한다.
3. Tag를 표시해 RX-28만 OPEN 950으로 한 번 이동하고 RX-64는 정지하는지 확인한다.
4. 주변 안전 확인 후에만 velocity guard를 열어 approach, `FINAL_APPROACH`,
   `STABILIZING`, `ALIGNED`를 진행한다. RX-28 CLOSE 350을 확인한다.
5. 기존 close wait 3초 후 RX-64가 천천히 raw 20(최고 위치)으로 이동하고 DONE이 되는지 확인한다.
6. temporary Tag loss를 만들어 unexpected gripper reposition이 없는지 확인한다.
7. `lift_enabled:=false`로 재실행해 OPEN/CLOSE 후 RX-64가 움직이지 않는지 확인한다.

#### Troubleshooting

- Tag인데 OPEN이 없으면 `/follower/supply/detected`, sequence node/status, raw command를
  순서대로 확인한다.
- ALIGNED인데 CLOSE가 없으면 `/follower/base_alignment/state`가 정확히 `ALIGNED`인지와
  sequence가 `OPENING` 상태였는지 확인한다.
- launch 직후 RX-28/RX-64가 움직이면 다른 standalone Dynamixel node, 이전 install,
  `startup_pose_enabled`와 `startup_torque` 값을 확인하고 즉시 actuator 전원을 안전하게
  차단한다.
- Tag loss 때 움직이면 `tag_lost_idle_enabled=false`와 실행 중인 sequence 중복 여부를
  확인한다.
- profile/ID가 다르면 두 node의 `robot=follower` parameter를 확인한다.
- U2D2 open error는 `/dev/ttyUSB*` 경로, 권한, 케이블, 다른 process의 port 점유를
  확인한다. Leader/Follower가 같은 기본 port를 동시에 열면 안 된다.
- duplicate node가 보이면 standalone `dynamixel_orin.launch.py` 또는
  `gripper_sequence.launch.py`를 종료하고 integrated launch 하나만 유지한다.
- topic이 다르면 위 표의 absolute names와 remap/namespace를 비교한다.
- `gripper_enabled=false`인데 node가 남으면 이전 launch process 또는 stale install을
  확인한다.
- `lift_enabled=false`인데 RX-64가 움직이면 raw/semantic topic의 외부 publisher와 다른
  Dynamixel process를 확인하고 regression으로 취급한다.
- lift를 켰는데 LIFTING이 없으면 `lift_raw`가 finite integer이며 configured range 안인지,
  CLOSE 후 `close_wait`가 지났는지 status에서 확인한다.
- RX-64가 너무 빠르면 `/follower/dynamixel_orin_node`의 `rx64_speed`가 `50`인지 확인한다.
- RX-64 status packet error는 port `/dev/ttyUSB0`, baudrate `115200`, `robot=follower`,
  ID `50` 및 케이블·전원을 확인한다.
- RX-28 속도가 달라졌다면 이번 변경에는 RX-28 Moving Speed write가 없으므로 regression
  또는 다른 Dynamixel process를 확인한다.
- lift target validation error는 실행 중 profile과 RX-64 range `20..270`을 확인한다.
- launch 직후 wheel이 움직이면 `/follower/velocity_guard/enable` 상태와 중복 motor
  publisher를 확인하고 guard를 즉시 닫는다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

정지하거나 시험을 마칠 때는 즉시 guard를 다시 닫습니다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

Pipeline 상태와 최종 bridge 연결은 다음 명령으로 확인합니다.

```bash
ros2 topic echo /follower/base_alignment/state
ros2 topic echo /follower/alignment/control_mode
ros2 topic echo /follower/alignment/command
ros2 topic echo /follower/approach/cmd_vel_raw
ros2 topic echo /follower/selected_cmd_vel
ros2 topic echo /follower/safe_cmd_vel
ros2 topic info /follower/safe_cmd_vel --verbose
```

STM32 I2C firmware가 준비된 뒤 수신 frame은 다음으로 확인합니다. I2C slave가 아직 ACK하지
않아 발생하는 timeout은 software launch integration 실패를 의미하지 않습니다.

```bash
ros2 topic echo /follower/stm32_rx/frame_count
```

Perception만 개별 실행하려면 기존 launch를 그대로 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

카메라 보정값과 AprilTag·접근 상태 파라미터는 모두 패키지의 `config/`에 포함되어
있으며, 기본 실행에는 작업공간 밖의 설정 파일이 필요하지 않습니다.

기존 파이프라인을 유지하고 상태 판단 노드만 실행할 때는 다음을 사용합니다.

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

영상 확인은 별도 터미널에서 실행합니다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```

## 사용자 최종 확인

- `docs/MANUAL_STATE_TEST.md`에 따라 실제 태그를 좌우·전후로 이동해 모든 상태를 확인합니다.
- `rviz2`의 Fixed Frame을 `base_link` 또는
  `follower/follower_camera_optical_frame`으로 설정하고 TF와
  `/follower/supply/relative_pose`를 사용자가 직접 확인합니다.
- camera state의 `target_distance=0.15 m`와 base/controller target `0.25 m`는 서로 다른
  software-validation 값이다. 둘 다 실제 그리퍼/TCP 기준 grasp 거리로 확정하지 않습니다.

RIGHT/TARGET/HIDDEN 실카메라 시나리오와 위 Dynamixel hardware sequence는 아직 사용자가
직접 확인해야 하며 문서에서 `NOT VERIFIED`로 유지합니다. STM32 I2C slave firmware와
motor algorithm은 이 launch 통합 작업의 구현 범위 밖입니다.
