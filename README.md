# 다중 로봇 재난 탐색·구호물품 운반 ROS 2 워크스페이스

`damgc_robot`은 소형 로봇 2대로 재난 환경을 탐색하고, 생존자 위치를 찾은 뒤
구호물품을 단독 또는 협동 운반하기 위한 ROS 2 Humble 워크스페이스입니다.

- 탐색·리더 로봇: RealSense D435 기반 인지, 3차원 지도, 자율주행, 임무 조정
- 운반 보조·팔로워 로봇: AprilTag 기반 상대 위치 보정과 협동 운반 지원
- 공통 하위 제어: STM32 기반 모터·엔코더·IMU·그리퍼 제어

최종 범위는 [개발 계획서](docs/Plan.md), 현재 완료 범위와 다음 작업은
[개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)을 기준으로 확인합니다.

## 현재 구현 상태

2026년 9월 18일 기준으로 저장소와 실제 Jetson + D435 실행에서 확인되는 구현은 다음과 같습니다.

- 리더: URDF/RViz 모델, D435 RGB·depth, RGB 보정, AprilTag 검출, 중앙 depth CSV 측정,
  exact-stamp TF2 기반 `base_link` pose·metric·상태, raw approach controller와
  velocity guard를 통한 최종 software topic `/leader/cmd_vel`, 독립
  [`rescue_robot_survivor`](src/leader/rescue_robot_survivor/README.md) Stage 1 YOLO11n
  person debug-image pipeline(Stage 1 VERIFIED: 실제 D435에서 1/2/3명 검출 및 debug image 확인),
  Stage 2 사람별 aligned-depth 거리(`IMPLEMENTED - HARDWARE VERIFICATION REQUIRED`)와 Stage
  3 CameraInfo.P 기반 camera optical XYZ/debug overlay/다중 `PoseArray`(실제 Jetson + D435
  `VERIFIED`, 2026-09-12)
- 팔로워: USB 카메라, AprilTag 검출, 기존 camera-frame 상태, exact-stamp TF2 기반
  `base_link` pose·metric·상태, raw approach controller, STOP/APPROACH/COOPERATION
  command selector, 최종 safety guard와 `/follower/safe_cmd_vel`
  - hybrid base 안정화는 `base_stable_time=0.30 s`와 fresh sample 3회 confirmation 사용
  - FINAL_APPROACH/STABILIZING tag loss는 0.30 s 동안 state/mode만 유지하고 velocity는 zero
  - blind final 기본값은 false이며 새 approach session마다 이전 ALIGNED latch reset
- 추가 구현: STM32 I2C/UART binary protocol, IMU·wheel state 수신, `/cmd_vel`
  전달, wheel odometry 원시 계산, 엔코더·IMU/VSLAM dual EKF 융합 설정, 모터
  속도 PID와 watchdog
- 확인됨: STM32 wheel/IMU 수신, dual EKF·Visual SLAM·nvblox 동시 실행,
  정지·저속 이동 상태 dual EKF/VSLAM 안정성, Docker RViz의 카메라 및 3차원 mesh 표시
- Survivor Stage 5 완료: YOLO person detection, aligned depth 거리,
  camera optical XYZ와 `/leader/survivor/camera_positions`, 원본 촬영 시각의
  camera→map TF2와 `/leader/survivor/map_positions`, sphere/text RViz 표시 및
  `/leader/survivor/map_markers` 검증 완료. D435 공유 상태에서 VSLAM·dual EKF·
  nvblox·Survivor 동시 실행과 nvblox 3D map/marker 동시 표시를 수동 확인했다.
- Survivor Stage 6 Persistent Survivor Registry — VERIFIED: typed `SurvivorTrack` interfaces, spatial one-to-one
  association, tentative→confirmed lifecycle, mission-runtime persistent ID, LOST/
  reassociation, EMA stabilization, reset service와 Registry RViz marker를 추가했다.
-  Stage 6.1 실제 수동 검증에서 여러 사람 distinct persistent ID, moving person same ID,
  FOV→LOST, same-ID reassociation, yellow LAST SEEN marker, white status text와
  Registry text Z+1.0 m를 확인했다. VSLAM·dual EKF·nvblox 위 RViz 표시까지
  **VERIFIED**다.
- 진행 중: 저속 주행에서 EKF 안정성과 VSLAM tracking 장시간 검증
- 협동 미션(2026-09-30 추가, `src/cooperative_mission`): 리더 AprilTag(QR) 탐색·정렬·파지 →
  팔로워 반대편 이동·파지 → ACK 기반 동시 리프트 → 공통 속도 1초 협동 직진을 조정하는 Mission
  Coordinator. 소프트웨어·시뮬레이션 시험 완료, **실물 검증 필요**. STM32 펌웨어 변경 없음.
- 아직 없음: Nav2, process/map-session 외부 영속 저장. `Survivor candidate N`은 현재 PoseArray 인덱스에
  따른 임시 번호다. Stage 4 raw map stability에는 약 0.115 m의 A→B 변화가 있어
  정밀 절대 위치 보장은 하지 않는다. 다음 생존자 개발은 association parameter 정확도
  검증과 장시간 안정성 평가다.

Leader AprilTag pipeline은 guarded `/leader/cmd_vel`에서 I2C STM32 bridge와 motor까지
통합되어 있습니다. Follower의 `/follower/safe_cmd_vel`은 아직 motor에 연결하지
않았습니다. 실제 이동 전에는 hardware E-stop과 bridge watchdog을 별도로 확인해야
합니다.

## 생존자 인식·지도·RViz 파이프라인 — Stage 5 PASS / Stage 6 Persistent Survivor Registry VERIFIED

공유 D435의 RGB와 aligned depth에서 YOLO가 사람을 검출하고 camera optical XYZ를
계산합니다. 검출 영상의 원본 timestamp로 TF2 camera→map 변환을 수행한 뒤,
RViz에 현재 후보의 sphere와 좌표 text를 표시합니다. 같은 D435의 infra1/infra2는
VSLAM에, RGB/depth는 nvblox 3D mapping에 사용됩니다. Wheel odometry와 IMU는
dual EKF를 거쳐 `map → odom → base_link` TF를 제공합니다.

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
sphere/text 제거도 확인했습니다. VSLAM, local/global EKF, nvblox와 세 survivor
topic이 동시에 동작했습니다. 저장소의 marker 기본 lifetime은 `2.0 s`이며
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

현재 공식 실행은 다음 3 terminal입니다. 첫 실행기가 D435, VSLAM, nvblox와
RViz를 시작하므로 카메라 실행기를 중복 기동하지 않습니다. 새 통합 launch의
3-terminal 정지 상태 동시 실행과 종료 격리는 확인했습니다. 실제 사람 관측 기반
Stage 6.1 회귀는 아직 별도 검증이 필요합니다.

```bash
# Terminal 1 — VSLAM + nvblox + RViz
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2 — survivor ROS pipeline
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_pipeline.launch.py

# Terminal 3 — YOLO detector
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

Raw 후보 marker를 끄려면 Terminal 2 명령에 `enable_raw_visualizer:=false`를
붙입니다. 전체 설계, 자동 검증 결과와 실물 검증 체크리스트는
[통합 launch 검증 문서](docs/SURVIVOR_PIPELINE_INTEGRATED_LAUNCH_VALIDATION.md)에 있습니다.

별도 ROS 환경 터미널에서 확인합니다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
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

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/damgc_robot
colcon build --symlink-install
source install/local_setup.bash
```

다른 경로에 clone했다면 `cd` 경로만 해당 저장소 루트로 바꿉니다. 두 Orin은
JetPack 6.2.2, Ubuntu 22.04, ROS 2 Humble과 동일한 의존성 버전을 사용하는 것이
계획의 기준입니다.

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
ros2 launch leader_approach_control velocity_guard.launch.py
```

Component 단독 launch에서는 controller와 guard가 모두 disabled로 시작하므로 enable
전에는 raw/final command가 zero입니다. 현재 hybrid 정렬은 FAR에서 Tag center를
추적하고 `0.40 m` 안에서 bounded tag-normal correction을 시작합니다. Pre-align
`0.30 m`, final target `0.20 m`는 실제 grasp 자세를 확인한 뒤 조정할 초기값입니다.
STM32/UART/motor에는 연결하지 않습니다.

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

## 협동 미션: QR 인식 → 파지 → 반대편 파지 → 동시 리프트 → 1초 협동 직진

`cooperative_mission` 패키지가 프로젝트 시나리오 전체를 자동으로 수행합니다. 기존 AprilTag
인식·접근 controller·command selector·velocity guard·STM32 bridge·Dynamixel 노드를 그대로
재사용하고, 리더 `mission_coordinator`와 팔로워 `mission_executor`만 추가했습니다.

```bash
bash scripts/run_cooperative_mission.sh follower   # Follower Orin 먼저
bash scripts/run_cooperative_mission.sh leader     # Leader Orin, Enter로 시작
```

1. 리더가 물체 태그를 제자리 탐색으로 찾고 정렬 → 2. 리더 집게 닫기 → 3. 팔로워가 반대 면 태그로
정렬 후 집게 닫기 → 4. 팔로워 ACK 순간 두 RX-64 동시 리프트 → 5. 리더 공통 속도를 팔로워가 부호
반전으로 추종해 같은 방향 1.0 s 직진 → 정지·유지. 이후 Enter로 함께 내려놓습니다.

설계·안전 정책은 [cooperative_mission README](src/cooperative_mission/README.md), 실행·보정 절차는
[협동 미션 실행 가이드](docs/COOPERATIVE_MISSION_RUN_GUIDE.md)를 따릅니다.

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

전체 topic ownership, enable 순서, 파라미터, 선택 빌드와 236개 자동시험 결과는
[Follower base-link velocity pipeline 검증 가이드](src/follower/follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다. Follower 실카메라 RIGHT/TARGET/HIDDEN 시나리오는 아직 `NOT VERIFIED`이며
사용자가 직접 확인해야 합니다.

상세 토픽과 상태 정의는
[리더·팔로워 구조](docs/LEADER_FOLLOWER_ARCHITECTURE.md)에서 확인할 수 있습니다.

## 문서

- [문서 안내](docs/README.md)
- [프로젝트 개요](docs/PROJECT_OVERVIEW.md)
- [개발 계획서](docs/Plan.md)
- [개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)
- [협동 미션 실행 가이드](docs/COOPERATIVE_MISSION_RUN_GUIDE.md)
- [1차 구현·시험 기록](docs/progress/week%201/README.md)
