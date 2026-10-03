# Leader Integration Phase 1–2 구현 기록

## 기록 정보

- 작업 날짜: 2026-10-02
- Baseline commit: `1c9394f6feef5e5d0c9ce1c7aa0c7a0693c18167`
- Baseline annotated tag: `pre_leader_scenario_v1_20261002`
- Tag target: baseline commit과 동일
- 시작 당시 repository: `main`, `origin/main`과 동기화, clean worktree

## 개발 목적

Phase 1은 Mapping 중 D435, camera processing, TF와 STM32 resource 중복 기동을 방지하고
기존 topic을 공유하도록 launch ownership을 선택 가능하게 한다. Phase 2는 Leader 최종
`/leader/cmd_vel` publisher를 하나로 만들고 `STOP/TELEOP/APPROACH/NAV2`, stale timeout과
mode transition zero를 제공한다.

VSLAM/nvblox/Survivor/AprilTag/gripper algorithm과 설정값은 수정하지 않았다. AprilTag
velocity guard의 output topic만 selector input으로 연결했다.

## Phase 1 — Shared resource integration

### D435, processing 및 TF owner

`run_vslam_mapping.sh`가 D435 RealSense driver를 실행한다. Survivor pipeline은 기존 camera
image/depth/CameraInfo를 구독하며 CameraInfo bridge와 RGB rectifier를 실행한다. VSLAM bringup은
robot model TF를 제공한다. AprilTag camera launch에 다음 argument를 추가했다.

| Argument | Standalone default | Mapping/Survivor 병행 |
|---|---:|---:|
| `start_camera` | `true` | `false` |
| `start_camera_processing` | `true` | `false` |
| `start_robot_state_publisher` | `true` | `false` |

각 조건은 해당 RealSense include, CameraInfo bridge/rectifier node, robot_state_publisher에
적용된다. AprilTag detector, tag TF와 alignment node는 계속 실행한다.

### STM32 bridge owner

Mapping script가 유일한 `/leader/stm32_bridge` owner다. `leader_apriltag_drive.launch.py`의
`use_stm32_bridge`는 standalone compatibility를 위해 기본 `true`; Mapping 공유 mode에서는
`false`다. STM32 node, topic, I2C address, firmware와 packet protocol은 수정하지 않았다.

### Phase 1 safety와 검증

Phase 1 단독 shared mode에는 `start_velocity_guard:=false`를 제공해 selector가 없는 상태에서
AprilTag가 final motor topic을 점유하지 않도록 했다. Phase 2 이후에는 Mapping selector를
재사용하고 guard는 startup-disabled로 실행한다. Mapping selector가 TELEOP인 동안 safe
APPROACH command는 선택되지 않는다.

별도 ROS domain에서 shared launch를 hardware-free로 기동해 camera/processing/TF/STM32/guard/
gripper owner가 생성되지 않고 AprilTag detector/alignment/raw path만 생성되는 것을 확인했다.
실제 D435, STM32, motor는 연결하지 않았다.

## Phase 2 — Leader Command Selector

### 구현 및 public API

Follower selector의 `source_mode` parameter, 한 source만 선택하는 구조, local monotonic receipt
time, stale timeout, Twist validation, mode cache clear와 transition zero를 적용했다. Follower
전용 COOPERATION input은 넣지 않았다.

```text
STOP      → 항상 zero
TELEOP    → /leader/teleop/cmd_vel
APPROACH  → /leader/approach/cmd_vel_safe
NAV2      → /nav2/cmd_vel
output    → /leader/cmd_vel
status    → /leader/command_selector/status (std_msgs/String, transient-local)
```

Mode 변경 API:

```bash
ros2 param get /leader/command_selector source_mode
ros2 param set /leader/command_selector source_mode STOP
ros2 param set /leader/command_selector source_mode TELEOP
ros2 param set /leader/command_selector source_mode APPROACH
ros2 param set /leader/command_selector source_mode NAV2
ros2 topic echo /leader/command_selector/status --qos-durability transient_local
```

Generic default는 `STOP`. Status는 `STOP`, `WAITING_<MODE>`, `ACTIVE_<MODE>`, `STALE_<MODE>`다.
mode 전환 시 source cache를 지우고 즉시 zero를 publish하며, 새로운 mode의 fresh command가
도착할 때까지 zero를 유지한다. stale, invalid/non-planar Twist, shutdown도 zero로 fail closed한다.
잘못된 mode parameter는 거부된다.

Timeout: TELEOP 0.30 s, APPROACH 0.35 s, NAV2 0.50 s. publish rate는 50 Hz이며 출력은
`linear.x`, `angular.z`만 채운다.

### Source remap 및 최종 연결

- `arrow_key_teleop.py`의 default output을 `/leader/teleop/cmd_vel`로 변경했다. 키 조작과 speed
  defaults는 그대로다.
- `run_vslam_mapping.sh`는 selector launch에 `source_mode:=TELEOP`을 명시하고 teleop input topic도
  명시한다. selector process는 기존 host process group cleanup으로 종료한다.
- AprilTag guard는 기존 raw input `/leader/approach/cmd_vel_raw`과 기존 safety algorithm을
  유지하며 `/leader/approach/cmd_vel_safe`를 발행한다.
- Nav2 controller remap `/nav2/cmd_vel`은 그대로다.
- STM32 bridge는 기존 namespace 상대 `cmd_vel`, 즉 `/leader/cmd_vel`을 구독한다.
- selector만 final `/leader/cmd_vel`을 발행한다. Leader cooperation node는 해당 topic을
  구독만 하고 `/follower/cmd_vel` 및 cooperation target을 발행한다.

Standalone AprilTag launch는 selector를 포함하고 `selector_mode:=APPROACH`로 시작한다. guard는
기존처럼 startup-disabled다. Mapping에 AprilTag를 추가하면 camera/processing/TF/bridge를
비활성화하고 `start_command_selector:=false`로 이미 실행된 selector를 공유한다.

## 변경 파일과 이유

- `src/leader/leader_command_selector/` — 새 selector package, launch/config, runtime status,
  README와 logic/node/config tests.
- `src/leader/leader_approach_control/config/velocity_guard.yaml` 및
  `leader_approach_control/velocity_guard_node.py` — safe output을 APPROACH selector input으로
  변경; guard algorithm 보존.
- `src/leader/leader_approach_control/test/test_configuration.py` — guard topic contract 검사.
- `src/leader/rescue_robot_bringup/scripts/arrow_key_teleop.py` — dedicated teleop topic default.
- `scripts/run_vslam_mapping.sh` — TELEOP selector 시작/cleanup, dedicated teleop topic, progress
  단계 표시.
- `src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py` — camera, processing, TF owner
  conditions.
- `src/leader/rescue_robot_bringup/launch/leader_apriltag_drive.launch.py` — selector launch/mode,
  guard/bridge condition, shared resource arguments.
- `src/leader/rescue_robot_bringup/package.xml` — selector runtime dependency.
- `src/leader/rescue_robot_bringup/test/test_leader_apriltag_drive_launch.py` — ownership, defaults,
  source path contract.
- `README.md`, `src/leader/leader_approach_control/README.md`,
  `src/leader/rescue_robot_bringup/docs/LEADER_APRILTAG_DRIVE_RUN_GUIDE.md` — topic/API 사용 설명 수정.
- `src/leader/rescue_robot_apriltag/docs/LEADER_*VALIDATION_GUIDE.md` 및
  `LEADER_FINAL_APPROACH_TAG_LOSS_AND_GRIPPER_INTEGRATION_GUIDE.md` — 과거 검증 내용은 보존하고
  현재 selector command path와 최신 실행 가이드 링크를 상단에 표시.
- `docs/LEADER_SCENARIO_V1_DEVELOPMENT_PLAN.md` — master roadmap.
- `docs/LEADER_INTEGRATION_PHASE1_2_2026-10-02.md` — 본 구현 기록.

## Build와 자동 테스트

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select \
  leader_command_selector leader_approach_control rescue_robot_bringup
colcon test --packages-select \
  leader_command_selector leader_approach_control rescue_robot_bringup
```

Build는 세 package에서 성공했다. 본 기록 작성 시점에 다음 결과를 확인했다.

- `leader_command_selector`: 38 passed
- `leader_approach_control`: 69 passed, 신규 guard topic configuration test 포함.
- `rescue_robot_bringup`: AprilTag launch contract 10 passed, Survivor pipeline launch contract
  4 passed.
- `rescue_robot_bringup`: 14 passed. Python compile, mapping script `bash -n`, `git diff --check` 통과.
- 최종 overlay에서 selector, Leader AprilTag drive, camera launch의 `--show-args` 확인.

별도 ROS domain의 dummy publisher/subscriber 검증에서 STOP, TELEOP, APPROACH, NAV2, unselected
source isolation, transition zero, stale zero, invalid mode rejection이 통과했다. Status에서
`WAITING_TELEOP`, `ACTIVE_TELEOP`, `STALE_TELEOP`, `STOP`, `WAITING_APPROACH`, `ACTIVE_APPROACH`,
`WAITING_NAV2`, `ACTIVE_NAV2`를 확인했다.

별도 hardware-free AprilTag launch에서는 detector, alignment, controller, selector, guard가
실행됐다. `/leader/approach/cmd_vel_safe` publisher는 guard 하나, subscriber는 selector 하나,
`/leader/cmd_vel` publisher는 selector 하나였다. guard disabled 상태의 final Twist는 zero였다.

## Jetson 수동 검증 명령

우선 motor write를 끄고 D435 Mapping, Survivor, AprilTag shared launch graph를 확인한다.

```bash
# Terminal 1: selector를 TELEOP으로 시작
cd ~/damgc_robot
STM32_I2C_WRITE_ENABLED=0 ./scripts/run_vslam_mapping.sh

# Terminal 2: Survivor
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/setup.bash
ros2 launch rescue_robot_bringup survivor_pipeline.launch.py

# Terminal 3: AprilTag가 existing camera, TF, selector, STM32를 공유
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py \
  start_camera:=false start_camera_processing:=false \
  start_robot_state_publisher:=false use_stm32_bridge:=false \
  start_command_selector:=false gripper_enabled:=false
```

```bash
ros2 node list | sort
ros2 topic info /leader/cmd_vel --verbose
ros2 topic info /leader/teleop/cmd_vel --verbose
ros2 topic info /leader/approach/cmd_vel_raw --verbose
ros2 topic info /leader/approach/cmd_vel_safe --verbose
ros2 topic info /nav2/cmd_vel --verbose
ros2 topic echo /leader/command_selector/status --qos-durability transient_local
ros2 param get /leader/command_selector source_mode
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic info /nvblox_node/mesh
ros2 topic echo --once /leader/survivor/map_positions
ros2 topic echo --once /leader/survivor/tracks --qos-durability transient_local
ros2 topic echo --once /leader/apriltag/detections
ros2 topic echo /leader/base_alignment/state
ros2 run tf2_ros tf2_echo map base_link
```

기대 graph는 D435 1개, `/leader/stm32_bridge` 1개, final publisher `/leader/command_selector`
1개다. teleop 실행 후 teleop source publisher 1개, AprilTag 실행 후 safe source publisher 1개를
확인한다. VSLAM odometry, nvblox mesh, Survivor Registry, AprilTag image/detection/alignment과
`map → odom → base_link → camera` TF를 함께 확인한다.

실제 wheel path 시험은 별도 감독 절차에서만 진행한다. wheels lifted, E-stop 접근성, bridge
`i2c_write_enabled` 값을 먼저 확인한다. 카메라/graph 회귀 중에는 guard를 enable하거나 mode를
APPROACH/NAV2로 바꾸지 않는다.

## 회귀 및 미검증 hardware 항목

- VSLAM/nvblox 알고리즘과 launch parameter는 변경하지 않았다. launch contract 검사는 통과했으나
  Jetson concurrent Mapping regression은 아직 수행하지 않았다.
- Survivor detector, Registry algorithm/topic/config는 변경하지 않았다. 기존 pipeline launch
  contract는 통과했으며 live YOLO/map/Registry 동시 회귀는 Jetson에서 해야 한다.
- AprilTag detector/alignment algorithm/threshold는 변경하지 않았다. hardware-free command graph는
  확인했지만 shared D435 실물 detection/alignment 회귀는 미수행이다.
- Gripper sequence, CLOSE/LIFT값과 trigger topic은 변경하지 않았다. launch contract는 통과했으나
  RX-28/RX-64 hardware sequence 회귀는 미수행이다.
- Nav2 mode는 dummy Twist로만 확인했다. NavigateToPose나 실제 wheel drive는 실행하지 않았다.
- 실제 STM32 velocity write, wheel motion, full mission은 수행하지 않았다.

## 구현하지 않은 기능

Survivor map pose→delivery goal, map/odom adapter, Supply Registry/waypoint, 물품 자동 접근,
Mapping 완료 trigger/Mission Coordinator, LOWER/OPEN delivery, Frontier exploration, Follower
cooperation, Heavy/Light branch는 구현하지 않았다.

## Known issues 및 다음 Phase 3

- Jetson에서 VSLAM+nvblox+Survivor+AprilTag shared D435 session을 동시 실행한 hardware regression이
남아 있다.
- `/leader/cmd_vel` final ownership을 유지하려면 selector 밖 direct publisher가 없어야 한다.
- 기존 guard timeout 0.30 s 뒤 selector APPROACH timeout 0.35 s가 적용된다. Phase 3 저속 wheel
  시험에서 연속 watchdog 정지 동작을 확인한다. 근거 없이 기존 safety timing을 바꾸지 않는다.
- Phase 3은 selector NAV2 dummy 검증 뒤 STM32 write를 별도 의도적으로 켜고, E-stop 및
  wheels-lifted 절차로 0.5 m/1 m/회전/장애물 stop을 계측하는 일이다. Nav2 planner/DWB,
  nvblox costmap과 delivery adapter는 그 phase의 수정 범위가 아니다.

## Rollback

```bash
git show --no-patch --oneline pre_leader_scenario_v1_20261002
```

Baseline tag/hash를 보존한다. 변경을 되돌릴 때는 검토된 해당 변경만 revert하거나 tag에서 별도
worktree/branch를 만든다. 전체 worktree reset/checkout을 사용하지 않는다.
