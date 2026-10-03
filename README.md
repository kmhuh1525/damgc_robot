# 다중 로봇 재난 탐색·구호물품 운반 ROS 2 워크스페이스

`damgc_robot`은 소형 로봇 2대로 재난 환경을 탐색하고, 생존자 위치를 찾은 뒤
구호물품을 단독 또는 협동 운반하기 위한 ROS 2 Humble 워크스페이스입니다.

- 탐색·리더 로봇: RealSense D435 기반 인지, 3차원 지도, 자율주행, 임무 조정
- 운반 보조·팔로워 로봇: AprilTag 기반 상대 위치 보정과 협동 운반 지원
- 공통 하위 제어: STM32 기반 모터·엔코더·IMU·그리퍼 제어

최종 목표와 장기 로드맵은 [개발 계획서](docs/Plan.md)와
[개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)을 참고합니다. 아래 상태 요약과
실행 방법은 저장소의 2026년 10월 2일 구현 및 검증 결과를 기준으로 합니다.

## 현재 구현 상태

2026년 10월 2일 기준으로 저장소와 Jetson + D435/Docker 실행에서 확인되는 구현은
다음과 같습니다.

- 리더 인지·접근: URDF/RViz, D435 RGB/depth, RGB 보정, AprilTag 검출, depth CSV 측정,
  exact-stamp TF2 기반 pose/metric/state, hybrid approach controller와 velocity guard를
  통한 `/leader/cmd_vel` software 경로가 구현되어 있다.
- 리더 그리퍼: `leader_apriltag_drive.launch.py`가 Dynamixel node와 gripper sequence를
  함께 실행한다. 기본값은 `gripper_enabled=true`, RX-28 open `1000`, close `480`,
  RX-64 lift `300`, speed `50`, `lift_enabled=true`이며 post-align odometry advance도
  활성화되어 있다. RX-64 raw `500→300` 이동은 hardware로 확인했지만 전체 물품 파지·운반
  성공은 아직 완료 조건으로 검증하지 않았다.
- 팔로워: USB 카메라, AprilTag 검출, 기존 camera-frame 상태, exact-stamp TF2 기반
  `base_link` pose·metric·상태, raw approach controller, STOP/APPROACH/COOPERATION
  command selector, 최종 safety guard와 `/follower/safe_cmd_vel`
  - hybrid base 안정화는 `base_stable_time=0.30 s`와 fresh sample 3회 confirmation 사용
  - FINAL_APPROACH/STABILIZING tag loss는 0.30 s 동안 state/mode만 유지하고 velocity는 zero
  - blind final 기본값은 false이며 새 approach session마다 이전 ALIGNED latch reset
- 하위 제어: STM32 I2C/UART binary protocol, IMU·wheel state, raw wheel odometry,
  `/cmd_vel` 전달, motor PID와 watchdog이 구현되어 있다. 과거 dual-EKF 경로도 검증했지만,
  현재 공식 `run_vslam_mapping.sh` 경로에서는 VSLAM이 `map→odom`과 `odom→base_link`
  TF를 직접 발행한다.
- VSLAM·nvblox·Nav2: 단일 D435, VSLAM, nvblox mesh/3D ESDF와 Nav2 planner/controller/
  BT navigator/lifecycle manager 통합이 구현되어 있다. Nav2 `ComputePathToPose` 경로
  생성과 `odom` frame은 확인했지만 `/nav2/cmd_vel`은 wheel bridge와 분리되어 있고,
  현장 controller가 유효 trajectory를 찾지 못해 autonomous driving 완료로 판정하지 않는다.
- Survivor Stage 5 완료: YOLO person detection, aligned depth 거리,
  camera optical XYZ와 `/leader/survivor/camera_positions`, 원본 촬영 시각의
  camera→map TF2와 `/leader/survivor/map_positions`, sphere/text RViz 표시 및
  `/leader/survivor/map_markers` 검증 완료. D435 공유 상태에서 VSLAM·nvblox·Survivor
  동시 실행과 nvblox 3D map/marker 동시 표시를 수동 확인했다.
- Survivor Stage 6 Persistent Survivor Registry — VERIFIED: typed `SurvivorTrack` interfaces, spatial one-to-one
  association, tentative→confirmed lifecycle, mission-runtime persistent ID, LOST/
  reassociation, EMA stabilization, reset service와 Registry RViz marker를 추가했다.
- Stage 6.1 실제 수동 검증에서 여러 사람 distinct persistent ID, moving person same ID,
  FOV→LOST, same-ID reassociation, yellow LAST SEEN marker, white status text와
  Registry text Z+1.0 m를 확인했다. VSLAM·nvblox 위 RViz 표시까지
  **VERIFIED**다.
- 협동 미션(2026-09-30 추가, `src/cooperative_mission`): 리더 태그 정렬·파지 → 팔로워 반대편 파지 → ACK 기반 동시 리프트 → 공통 속도로 1초 협동 직진을 조정한다. 소프트웨어·시뮬레이션 구현은 있으나 실물 검증이 남아 있다.
- 아직 완료되지 않음: Nav2 autonomous driving, 전체 물품 파지·운반 검증, 실물 리더–팔로워 협동 운반, process/map-session 외부 Survivor ID 저장.
- Survivor의 `Survivor candidate N`은 현재 PoseArray frame-local 번호이고 Registry ID와
  다르다. Stage 4 raw map A→B 평균 변화는 약 `0.115 m`였으므로 정밀 절대 위치를
  보장하지 않는다. 남은 Survivor 작업은 aligned-depth 거리의 정량 하드웨어 검증,
  association parameter 검증과 장시간 안정성 평가다.

Leader AprilTag pipeline은 guarded `/leader/cmd_vel`에서 I2C STM32 bridge와 motor까지
통합되어 있습니다. Gripper는 leader 통합 launch에 포함되지만 전체 파지·운반은 별도
hardware 검증이 필요합니다. Follower의 `/follower/safe_cmd_vel`은 아직 motor에 연결하지
않았습니다. 실제 이동 전에는 hardware E-stop과 bridge watchdog을 별도로 확인해야 합니다.

## 생존자 인식·지도·RViz 파이프라인 — Stage 5 PASS / Stage 6.1 Registry VERIFIED

공유 D435의 RGB와 aligned depth에서 YOLO가 사람을 검출하고 camera optical XYZ를
계산합니다. 검출 영상의 원본 timestamp로 exact-time TF2 camera→map 변환을 수행한 뒤,
RViz에 현재 후보의 sphere와 좌표 text를 표시합니다. 같은 D435의 infra1/infra2는
VSLAM에, RGB/depth는 nvblox 3D mapping에 사용됩니다. 현재 공식 mapping 경로에서는
VSLAM이 `map → odom → base_link` dynamic TF를 발행하고 robot_state_publisher와
RealSense가 camera chain을 연결합니다. Nav2와 nvblox costmap은 `odom` frame을
사용하며 Survivor map registration은 `map` frame을 사용합니다.

```text
RGB + aligned depth + CameraInfo → YOLO → camera optical XYZ
  → /leader/survivor/camera_positions
  → exact-timestamp TF2 camera → map
  → /leader/survivor/map_positions
  → Survivor map visualizer → /leader/survivor/map_markers
  → RViz sphere + text, nvblox 3D map과 동시 표시
```

실제 Jetson + D435 수동 검증에서 단일·다중 후보와 text 표시, nvblox mesh와
marker의 동시 표시, 같은 timestamp의 map pose와 sphere 좌표 일치를 확인했습니다.
Stage 5 Raw Visualizer는 사람 후보가 FOV 밖으로 나간 뒤 marker가 finite lifetime 이후 사라지고, 재진입하면
다시 생성되는 것을 확인했습니다. 통제된 2명→1명 감소에서는 이전 후보의
sphere/text 제거도 확인했습니다. VSLAM, nvblox, Nav2와 세 survivor topic이
동시에 동작했습니다. 저장소의 marker 기본 lifetime은 `2.0 s`이며
Raw Visualizer의 text 표시 Z offset은 `0.30 m`입니다. offset은 원본 map XYZ를 바꾸지
않습니다. Stage 6 Registry Visualizer는 sphere를 filtered map position에 유지하고
Registry text만 `1.0 m` 위에 표시하며, LOST는 노란색 불투명 sphere와 흰색 LAST SEEN
text로 표시합니다. 두 visualizer branch는 서로 독립적입니다.

`Survivor candidate 1/2`는 현재 `PoseArray` 인덱스에 따른 임시 번호입니다.
Persistent Registry는 이 raw branch와 분리되어 다음 topic을 제공합니다.

```text
/leader/survivor/map_positions
        ↓ survivor_registry_node
/leader/survivor/tracks
        ↓ survivor_registry_visualizer_node
/leader/survivor/registry_markers
```

Stage 6.1은 temporal confirmation, 다중 인물 distinct ID, 이동 중 same-ID, FOV→LOST,
마지막 위치 유지, same-ID 재진입과 Registry visualization을 실제 환경에서 검증했다.
process/map-session 외부 영속 저장과 CSV/JSON 저장은 구현하지 않는다.
Stage 2의 aligned-depth 거리는 실제 파이프라인에서 확인됐지만, 별도 줄자 기준
거리표 검증은 완료되지 않았습니다.

현재 공식 Survivor 실행은 다음 2 terminal입니다. 첫 실행기가 D435, VSLAM,
nvblox와 RViz를 시작하므로 카메라 실행기를 중복 기동하지 않습니다. 통합 launch가
YOLO detector와 survivor ROS 노드, `rqt_image_view`를 함께 시작합니다.

```bash
# Terminal 1 — VSLAM + nvblox + RViz
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2 — integrated survivor pipeline (YOLO + survivor nodes + image view)
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rescue_robot_bringup survivor_pipeline.launch.py
```

Raw 후보 marker를 끄려면 `enable_raw_visualizer:=false`를, debug GUI를 끄려면
`show_image_view:=false`를 붙입니다. 통합 launch가 detector를 이미 실행하므로
`./scripts/run_survivor_detector.sh`를 동시에 실행하지 마십시오. 이 script는
수동 fallback/debug 용도로 유지됩니다. 전체 설계, 자동 검증 결과와 실물 검증 체크리스트는
[통합 launch 검증 문서](docs/SURVIVOR_PIPELINE_INTEGRATED_LAUNCH_VALIDATION.md)에 있습니다.

별도 ROS 환경 터미널에서 확인합니다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --once /leader/survivor/camera_positions
ros2 topic echo --once /leader/survivor/map_positions
ros2 topic echo --once /leader/survivor/map_markers
ros2 topic info -v /leader/survivor/map_markers
ros2 topic echo --once /leader/survivor/tracks --qos-durability transient_local
ros2 topic info -v /leader/survivor/tracks
ros2 topic echo --once /leader/survivor/registry_markers --qos-durability transient_local
ros2 service call /leader/survivor/registry/reset std_srvs/srv/Trigger "{}"
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /leader/odometry/local
ros2 topic hz /leader/odometry/global
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info -v /nvblox_node/mesh
ros2 service list | rg /nvblox_node/get_esdf_and_gradient
```

사람이 움직일 때 좌표를 비교하려면 각 topic을 별도 `echo --once`로 읽은 값을
서로 짝짓지 말고, 동일한 원본 timestamp의 map pose와 marker를 비교합니다.
현재 3D ESDF mode에서는 `/nvblox_node/static_esdf_pointcloud` 무출력을 실패로
판정하지 않습니다. nvblox mesh, ESDF service와 RViz 3D map을 확인합니다.
측정값과 전체 수동 검증 절차는 [Stage 5 validation](docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE5_RVIZ_VISUALIZATION_VALIDATION.md)에
기록했습니다.

## VSLAM·nvblox·Nav2 현재 상태

`./scripts/run_vslam_mapping.sh`는 현재 하나의 D435 입력을 RealSense, VSLAM,
nvblox와 RViz가 공유하도록 실행합니다. 공식 mapping launch의 TF 소유권은 다음과
같습니다.

```text
VSLAM                  map → odom, odom → base_link
robot_state_publisher  base_link → camera_link
RealSense              camera_link → camera optical frames
```

nvblox는 `odom` 기준 mesh/3D ESDF와 costmap slice를 제공하고, Nav2는 planner,
controller, BT navigator와 lifecycle manager를 실행합니다. Nav2 global/local costmap은
`odom` frame과 nvblox layer를 사용하며, 로컬 costmap 호환 topic은
`/costmap/costmap`입니다.

현재 검증 범위는 다음과 같습니다.

- Nav2 lifecycle node 활성화
- `ComputePathToPose` 경로 생성
- `/plan`의 `odom` frame 확인
- nvblox mesh와 ESDF service 확인
- `/nav2/cmd_vel` 발행 확인

`/nav2/cmd_vel`은 Leader selector의 `NAV2` input으로 연결되어 있다. Mapping 시작 시
selector는 `TELEOP`이며 실제 Nav2 실차 주행은 수행하지 않았다. 따라서 현재 상태를
완성된 autonomous driving으로 표시하지 않으며, 실장비 이동·반복
목표 주행·controller valid trajectory 확보가 남아 있습니다. 상세 절차와 2026-10-02
검증 결과는 [nvblox/Nav2 검증 문서](src/leader/rescue_robot_bringup/docs/NVBLOX_NAV2_RVIZ.md)를
참고합니다.

기본 상태 확인 명령:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list -t | rg navigate_to_pose
ros2 topic echo --once /plan --field header.frame_id
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic info /nvblox_node/static_map_slice
ros2 topic info /nav2/cmd_vel
```

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/damgc_robot
colcon build --symlink-install
source install/setup.bash
```

다른 경로에 clone했다면 `cd` 경로만 해당 저장소 루트로 바꿉니다. JetPack
6.2.3/L4T R36.5.2, Ubuntu 22.04, ROS 2 Humble과 검증된 survivor Docker runtime
의존성 조합을 기준으로 합니다. Host global Python에 PyTorch/Ultralytics를 설치하지
않고 `damgc-survivor-yolo:humble` image를 사용합니다.

실제 장비를 사용할 때는 RealSense D435를 Orin에 연결한 뒤 다음 명령으로 장치가
인식되는지 먼저 확인합니다.

```bash
rs-enumerate-devices
```

## URDF만 확인

```bash
ros2 launch rescue_robot_description display.launch.py
```

## 리더 카메라 + URDF + image_proc + AprilTag 통합 실행

실제 D435가 연결된 상태에서 실행합니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
```

이 launch는 `/leader/camera` 아래에 RealSense RGB/depth 토픽을 만들고,
URDF TF, RGB 보정, CameraInfo QoS bridge, `/leader/apriltag/apriltag`를 함께 실행합니다.

USB 2.x 대역폭 때문에 RGB만 먼저 확인하려면:

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py enable_depth:=false
```

## Detection/TF 확인

```bash
ros2 topic echo --once /leader/apriltag/detections
ros2 run tf2_ros tf2_echo camera_color_optical_frame 'leader/tag36h11:0'
```

현재 `apriltag_leader.yaml`은 tag36h11 ID 0의 TF child frame을
`leader/tag36h11:0`으로 명시합니다. 과거 비접두 `tag36h11:0` 이름을 사용하지 않습니다.

## Depth CSV 측정

통합 launch를 실행한 뒤 별도 터미널에서 실행합니다.

```bash
ros2 run rescue_robot_tools depth_to_csv.py
```

기본 구독 토픽은 `/leader/camera/depth/image_rect_raw`입니다. 코드의 기본 저장 위치는
`~/jisu_ws/data/depth_distance.csv`이므로, 이 저장소의 `data` 폴더에 저장하려면 다음처럼
경로를 명시합니다.

```bash
ros2 run rescue_robot_tools depth_to_csv.py --ros-args \
  -p output_path:=/home/maze/damgc_robot/data/depth_distance.csv
```

## 리더 확인 명령

```bash
ros2 node list
ros2 topic list | grep leader
ros2 topic echo --once /leader/apriltag/detections
ros2 run rqt_image_view rqt_image_view /leader/camera/color/image_rect
ros2 run tf2_ros tf2_echo camera_color_optical_frame 'leader/tag36h11:0'
```

AprilTag 확인 시 실제 태그 ID 0과 5 cm 크기의 `tag36h11` 태그를 카메라 앞에
두어야 검출 결과와 태그 TF가 출력됩니다.

## 리더 AprilTag base-link velocity pipeline

실제 Leader 주행은 다음 통합 launch를 사용합니다. Approach controller는 자동으로
enabled되지만 velocity guard는 disabled로 시작하므로, 안전 확인 후 사용자가 guard를
명시적으로 enable하기 전까지 motor command는 zero로 유지됩니다.

```bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

이 통합 launch는 AprilTag 접근·velocity guard·STM32 경로와 함께 선택 가능한
Dynamixel/gripper sequence도 실행합니다. 기본 gripper gate는 켜져 있지만 velocity
guard는 별도로 enable해야 실제 바퀴가 움직입니다. gripper-only 또는 안전한 software
회귀는 `gripper_enabled:=false`로 실행할 수 있습니다. 현재 gripper 설정과 post-align
odometry 검증 절차는 [Leader AprilTag Drive Run Guide](src/leader/rescue_robot_bringup/docs/LEADER_APRILTAG_DRIVE_RUN_GUIDE.md)를
따릅니다.

빌드, 상태 확인, 주행 시작·정지와 I2C troubleshooting은
[Leader AprilTag Drive Run Guide](src/leader/rescue_robot_bringup/docs/LEADER_APRILTAG_DRIVE_RUN_GUIDE.md)를
따릅니다.

아래 명령은 STM32를 연결하지 않고 각 software component를 따로 시험할 때 사용합니다.

기본 `camera_apriltag.launch.py`는 detection까지만 실행하며, 기존 camera-frame 상태와
신규 base-link pose·metric·상태까지 함께 실행하려면 `enable_approach:=true`를 지정합니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=false enable_approach:=true
ros2 launch leader_approach_control approach_controller.launch.py
ros2 launch leader_command_selector command_selector.launch.py source_mode:=APPROACH
ros2 launch leader_approach_control velocity_guard.launch.py
```

Component 단독 launch에서는 controller와 guard가 모두 disabled로 시작하므로 enable
전에는 raw/safe command가 zero입니다. Selector는 final `/leader/cmd_vel`을 단독 발행하며,
실제 주행 시 guard enable과 selector `APPROACH` mode를 모두 확인해야 합니다. 현재 hybrid 정렬은 FAR에서 Tag center를
추적하고 `0.40 m` 안에서 bounded tag-normal correction을 시작합니다. 통합 leader
launch의 현재 기본값은 pre-align `0.30 m`, visual final target `0.23 m`, post-align
grasp target `0.20 m`입니다. `post_align_odom_enabled=true`이면 visual alignment 후
최대 `0.12 m`·8초 제한의 odometry advance가 수행됩니다. STM32/UART/motor를 사용하지
않는 component 단독 시험에서는 실제 주행이 발생하지 않습니다.

전체 topic, state priority, enable 순서, 파라미터와 실기·자동시험 결과는
[Leader velocity pipeline 검증 가이드](src/leader/rescue_robot_apriltag/docs/LEADER_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다.
Hybrid 설계 원리와 side-looking/FOV regression의 현행 절차는
[Leader Hybrid 알고리즘](src/leader/rescue_robot_apriltag/docs/LEADER_HYBRID_TAG_ALIGNMENT_ALGORITHM.md) 및
[Leader Hybrid 정렬 검증 가이드](src/leader/rescue_robot_apriltag/docs/LEADER_HYBRID_TAG_ALIGNMENT_VALIDATION_GUIDE.md)를
따릅니다.

## 리더 DDS 협력 통신

리더 Orin에서 별도 터미널로 실행합니다. 기본 상태는 정지(`IDLE`)입니다.

```bash
ros2 launch leader_cooperation leader_cooperation.launch.py
ros2 service call /cooperation/enable std_srvs/srv/SetBool "{data: true}"
```

팔로워가 `/follower/status` (`std_msgs/msg/String`) heartbeat를 발행하고,
리더의 `/leader/cmd_vel`이 들어오는 동안에만 `/follower/cmd_vel`로 전달됩니다.
heartbeat 또는 명령이 끊기면 0 속도로 정지합니다. 상세 계약은
[leader_cooperation README](src/leader/leader_cooperation/README.md)를 참고합니다.

기존 방향키 이동, 역할별 STM32 bridge와 DDS 연결을 한 번에 기동하는 전용 실행기는
팔로워에서 먼저, 리더에서 다음 순서로 실행합니다. 기본적으로 팔로워 velocity guard는
닫혀 있어 별도 enable 전에는 움직이지 않습니다.

```bash
./scripts/run_cooperative_transport.sh follower
./scripts/run_cooperative_transport.sh leader
```

모터 없는 네트워크 점검과 실제 장비의 안전한 enable/종료 순서는
[협동 이동 실행 가이드](docs/COOPERATIVE_TRANSPORT_RUN_GUIDE.md)를 따릅니다.

## 팔로워 인식 파이프라인

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

기존 카메라·AprilTag 파이프라인을 유지하고 상태 판정 노드만 실행할 때는:

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

전체 software velocity pipeline은 별도 터미널에서 controller, selector와 integrated guard를
실행합니다. Controller와 guard는 기본적으로 disabled이고 selector는 `STOP`으로 시작합니다.

```bash
ros2 launch follower_approach_control approach_controller.launch.py
ros2 launch follower_command_selector command_selector.launch.py
ros2 launch follower_control selected_velocity_guard.launch.py
```

AprilTag 접근 source를 사용할 때는 selector를 명시적으로 `APPROACH`로 전환한 뒤 controller와
guard를 각각 enable합니다. 기존 cooperation command는 `/follower/cmd_vel` 입력으로 유지되며
AprilTag controller가 이 토픽을 직접 publish하지 않습니다. 현재 base/controller target
`0.25 m`는 Leader와 맞춘 software-validation 값이지 실제 grasp 거리 확정값은 아닙니다.

전체 topic ownership, enable 순서, 파라미터와 자동시험 결과는
[Follower base-link velocity pipeline 검증 가이드](src/follower/follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다. Follower 실카메라 RIGHT/TARGET/HIDDEN 시나리오는 아직 `NOT VERIFIED`이며
사용자가 직접 확인해야 합니다.

상세 토픽과 상태 정의는
[리더·팔로워 구조](docs/LEADER_FOLLOWER_ARCHITECTURE.md)에서 확인할 수 있습니다.

## 협동 미션: 태그 인식 → 파지 → 동시 리프트 → 운반

`cooperative_mission`은 기존 인식·접근·안전 제어를 재사용하고 리더 `mission_coordinator`와 팔로워 `mission_executor`가 작업 순서를 맞춥니다.

```bash
bash scripts/run_cooperative_mission.sh follower   # Follower Orin 먼저
bash scripts/run_cooperative_mission.sh leader     # Leader Orin, Enter로 시작
```

리더가 태그를 찾아 정렬하고 집게를 닫은 뒤, 팔로워가 물체 반대편 태그를 찾아 정렬·파지합니다. 팔로워의 ACK를 받은 다음 양쪽 RX-64를 들어 올리고, 리더가 공통 속도를 발행하면 팔로워가 마주 보는 좌표계에 맞춰 방향 부호를 변환해 1초간 함께 직진합니다. 링크 단절이나 단계 timeout 시 정지하고 파지를 유지합니다.

설계·안전 정책은 [cooperative_mission README](src/cooperative_mission/README.md), 실기기 실행 절차는 [협동 미션 실행 가이드](docs/COOPERATIVE_MISSION_RUN_GUIDE.md)를 참고합니다. 실제 하드웨어 운반은 별도 검증이 필요합니다.

## 문서

- [문서 안내](docs/README.md)
- [프로젝트 개요](docs/PROJECT_OVERVIEW.md)
- [개발 계획서](docs/Plan.md)
- [개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)
- [협동 미션 실행 가이드](docs/COOPERATIVE_MISSION_RUN_GUIDE.md)
- [Survivor 통합 launch 검증](docs/SURVIVOR_PIPELINE_INTEGRATED_LAUNCH_VALIDATION.md)
- [nvblox/Nav2 검증](src/leader/rescue_robot_bringup/docs/NVBLOX_NAV2_RVIZ.md)
- [Leader AprilTag Drive 실행 가이드](src/leader/rescue_robot_bringup/docs/LEADER_APRILTAG_DRIVE_RUN_GUIDE.md)
- [1차 구현·시험 기록](docs/progress/week%201/README.md)
