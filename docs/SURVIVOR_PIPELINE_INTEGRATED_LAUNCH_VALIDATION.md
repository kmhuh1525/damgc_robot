# Survivor Pipeline 통합 Launch 개발·검증

> 기준: `main`, 시작 commit `f088b5d2a6335604a3f2707eed8332b181e9ec98`  
> 상태: 통합 launch 구현 기준 문서. Docker detector와 optional `rqt_image_view`를
> 포함한 2-terminal 실행 구조로 갱신 중이며 실제 사람 관측 회귀는 수동 검증 대상이다.
> 사람 관측이 필요한 Stage 6.1 실물 회귀는 사용자 수동 검증으로 남긴다 (**NOT RUN**).

## 1. 목적과 개발 배경

Stage 1~6.1 동안 survivor camera preprocessing, YOLO/Camera XYZ, 원본 촬영 시각의
Camera→Map TF2, raw map marker, Persistent Registry와 Registry marker를 별도 launch로
개발했다. 독립 terminal은 기능별 오류 추적에 유리했고, Stage 6.1의 다중 인물 ID,
FOV 이탈→LOST, 재연결, 시각화는 Jetson+D435 환경에서 이미 검증되었다. 당시 검증 범위는
[Stage 6.1 기록](SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_1_REGISTRY_ROBUSTNESS_VALIDATION.md)에 있다.

기능이 안정화된 뒤에도 기존 방식은 mapping, preprocessing, YOLO, map transform,
raw visualizer, Registry를 위해 여러 terminal을 요구했다. 이제 검증된 기능을
재작성하지 않고 survivor launch 하나가 ROS survivor 노드, Docker detector와 optional
debug GUI를 함께 조정한다.

## 2. Before / After와 단계적 통합 결정

| 이전 | 현재 |
| --- | --- |
| T1 `run_vslam_mapping.sh` | T1 `run_vslam_mapping.sh` |
| T2~T6 개별 survivor launch와 detector | T2 `survivor_pipeline.launch.py` |

Mapping 실행기는 STM32, D435 단일 소유권, VSLAM, dual EKF, nvblox, RViz와 rosbag
cleanup을 함께 관리한다. 통합 survivor launch는 기존 script와 동일한 Docker
runtime을 process action으로 실행하므로 NVIDIA runtime, host network/IPC, ROS 환경,
model cache, Docker workspace와 detector 설정을 그대로 사용한다. launch에서는
non-interactive process compatibility를 위해 Docker TTY flag만 제외한다. `rqt_image_view`는
`show_image_view:=true`가 기본이며 `false`로 끌 수 있다.

## 3. Package placement와 dependency

`rescue_robot_bringup`이 기존 통합 launch와 survivor camera preprocessing을 소유한다.
따라서 새 launch를 같은 package에 설치한다. `rescue_robot_bringup/package.xml`에
`rescue_robot_survivor` 실행 의존성을 추가했다. `rescue_robot_survivor`에서
`rescue_robot_bringup`으로 향하는 역방향 의존성은 생기지 않는다.
`CMakeLists.txt`의 `install(DIRECTORY launch ...)`가 새 파일을 설치한다.

## 4. Integrated architecture와 single D435 ownership

```text
Terminal 1: run_vslam_mapping.sh
  STM32 bridge ── wheel/IMU ── dual EKF ── map→odom→base_link TF
  D435 (단일 driver) ── RGB/CameraInfo/aligned depth ───────────┐
       ├─ infra ── Visual SLAM ── map TF                        │
       └─ mapping inputs ── nvblox mesh/3D ESDF ── RViz         │
                                                                │
Terminal 2: survivor_pipeline.launch.py                         │
  CameraInfo QoS bridge + RGB rectify ◀─────────────────────────┤
       └─ /leader/camera/color/image_rect ──────────────────┐   │
  Docker YOLO / Camera XYZ ◀───────────────────────────────┤   │
       └─ /leader/survivor/debug_image + camera_positions  │   │
  survivor_map_transform ◀── camera_positions ───────────┐ │   │
       └─ /leader/survivor/map_positions                   │ │   │
            ├─ raw visualizer ── map_markers               │ │   │
            └─ Registry ── tracks ── registry visualizer   │ │   │
                                   └─ registry_markers    │ │   │
                                                          │ │   │
  rqt_image_view (optional) ◀── /leader/survivor/debug_image
```

새 launch에는 `realsense2_camera`, `robot_state_publisher`, VSLAM, EKF, nvblox,
STM32, RViz, Nav2가 없다. D435는 Terminal 1만 시작한다. Terminal 2는 기존 RGB와
CameraInfo를 구독하고 기존 Docker detector를 실행한다. YOLO는 보정 RGB, aligned
depth, 원본 CameraInfo를 사용한다.

## 5. Child launch와 node ownership

| Child launch | 실행 node | 입력 → 출력 |
| --- | --- | --- |
| `rescue_robot_bringup/survivor_camera_processing.launch.py` | `/survivor_camera_info_qos_bridge`, `/survivor_color_rectify` | RGB·CameraInfo → `/leader/camera/color/image_rect` |
| `rescue_robot_survivor/survivor_map_transform.launch.py` | `/leader/survivor_map_transform` | `/leader/survivor/camera_positions` → `/leader/survivor/map_positions` |
| `rescue_robot_survivor/survivor_map_visualizer.launch.py` | `/leader/survivor_map_visualizer` | map_positions → `/leader/survivor/map_markers` (현재 후보) |
| `rescue_robot_survivor/survivor_registry.launch.py` | `/leader/survivor_registry`, `/leader/survivor_registry_visualizer` | map_positions → `/leader/survivor/tracks` → `/leader/survivor/registry_markers` |

Map transform은 유효한 camera positions에 대해 입력 timestamp의 TF를 조회하고
같은 stamp로 map PoseArray를 발행한다. TF가 없으면 해당 sample을 건너뛴다.
Raw visualizer의 후보 번호는 현재 프레임의 index이며, Registry의 ID는 해당
mapping session에서 유지되는 ID다. Map transform 내부 TF listener node가
ROS graph에 추가로 보일 수 있으나 transform 복제는 아니다.

| Component | 실행 소유자 |
| --- | --- |
| D435, STM32, VSLAM, dual EKF, nvblox, RViz | Terminal 1 mapping 실행기 |
| QoS bridge, rectify, map transform, raw marker, Registry, Registry marker | Terminal 2 통합 launch |
| YOLO, aligned depth association, Camera XYZ, `rqt_image_view` | Terminal 2 통합 launch |

## 6. Parameter ownership와 startup

통합 launch의 공개 인자는 `enable_raw_visualizer:=true`와
`show_image_view:=true`다. `enable_raw_visualizer:=false`에서는 raw visualizer만
빠지고 Registry와 Registry marker는 계속 실행된다. `show_image_view:=false`에서는
GUI만 빠지고 detector와 survivor pipeline은 계속 실행된다.
각 include는 독립 launch configuration scope에 배치한다. 이는 child launch들이
공유하는 `input_topic`, `output_topic`, `text_z_offset` 인자명이 서로의 기본값을
덮어쓰지 못하게 한다. 추가 상위 parameter 기본값은 없다.

Map transform의 `target_frame=map`, `tf_timeout_sec=0.2` 및 topic 기본값은
`survivor_map_transform.launch.py` 소유다. Stage 6.1의 `association_radius_m=0.50`,
`reassociation_radius_m=0.75`, `confirm_min_duration_sec=2.0`, `confirm_min_hits=4`,
`tentative_max_gap_sec=0.8`, `visible_timeout_sec=4.0`, `position_ema_alpha=0.50`,
`registry_publish_hz=2.0`, Registry text `text_z_offset=1.0`은
`survivor_registry.yaml`과 해당 child launch 소유다.

권장 시작 순서는 mapping → survivor pipeline이다. Subscriber는 upstream publisher가
생길 때까지 대기하며, startup `sleep`/`TimerAction`은 없다. 통합 launch 실행 중에는
`run_survivor_detector.sh`를 별도로 실행하지 않는다.

## 7. 변경 파일과 보호 범위

변경: `src/leader/rescue_robot_bringup/launch/survivor_pipeline.launch.py`,
`src/leader/rescue_robot_bringup/test/test_survivor_pipeline_launch.py`,
`src/leader/rescue_robot_bringup/package.xml`, root `README.md`, 본 문서.

`run_vslam_mapping.sh`, 네 child launch,
`person_detector.launch.py`, `survivor_registry.yaml`, survivor node/core/test는
변경하지 않았다. 작업 전부터 수정 상태였던 사용자 문서 변경은 보존했다.

## 8. Build, install-space discovery와 자동 테스트

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select rescue_robot_survivor rescue_robot_bringup
source install/local_setup.bash
ros2 pkg prefix rescue_robot_bringup
ros2 launch rescue_robot_bringup survivor_pipeline.launch.py --show-args
colcon test --packages-select rescue_robot_survivor rescue_robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

실제 결과: build **PASS**, prefix
`/home/maze/damgc_robot/install/rescue_robot_bringup`, installed launch discovery
**PASS**, `--show-args`에서 `enable_raw_visualizer` 기본값 `true` 확인.
선택 package 테스트는 survivor **114/114**, bringup **7/7**로 **121 passed, 0 failed**.
`colcon test-result --verbose`는 workspace에 남아 있는 결과를 합산하여
`669 tests, 0 errors, 0 failures, 0 skipped`로 표시했다. 669를 이번 선택 package의
테스트 수로 해석하지 않는다.

새 테스트는 설치된 child 경로, 각 child가 정의하는 정확한 node 집합,
독립 launch scope, raw visualizer true/false 조건, Registry branch 유지 여부를 확인한다.
이 node 집합 검증은 RealSense/SLAM/nvblox/YOLO/RViz/debug GUI가 include되지
않았음도 검사한다. 기존 Stage 6.1 테스트는 그대로 통과했다.

## 9. Terminal 2 단독 smoke와 종료 관찰

실행 전 `ros2 node list`는 비어 있었다. 통합 launch 단독 실행에서 위 6개 process가
시작했고, `/leader/person_detector` 및 RealSense node는 나타나지 않았다. TF listener
helper node는 하나 보였다. Upstream camera/YOLO/TF 없이도 실행 중인 node가
crash하지 않았다. `enable_raw_visualizer:=false`로 다시 실행했을 때 raw visualizer
process만 빠지고 5개 process가 시작했다. 두 경우 모두 Ctrl+C 후 `ros2 node list`에
남은 node가 없었다.

추가 standalone graph probe에서 `/leader/survivor/map_positions`는 transform
publisher 1개와 raw visualizer·Registry subscriber 각 1개로 연결되었다.
`/leader/survivor/map_markers`, `/leader/survivor/tracks`,
`/leader/survivor/registry_markers`에는 각각 기대한 publisher 1개가 있었다.
`tracks`의 publisher/subscriber는 모두 `TRANSIENT_LOCAL`이었다.
`ros2 param get`에서 Registry `association_radius_m=0.5`, Registry visualizer
`text_z_offset=1.0`을 확인했다. `/leader/survivor/registry/reset`은
`std_srvs/srv/Trigger`였고 실제 호출에 `success=True`, `next ID is 1`을 반환했다.
이어 받은 `tracks` message는 `frame_id: map`, `tracks: []`였다. 입력 검출 없이
수행했으므로 기존 ID·marker 삭제 동작의 실물 검증은 아니다.

종료 시 기존 `rescue_robot_apriltag/scripts/camera_info_qos_bridge.py`가 SIGINT 후
`rclpy.shutdown()`을 다시 호출해 `RCLError: rcl_shutdown already called`와 exit code 1을
출력했다. 다른 5개 child는 clean exit였다. 동일 현상은 기존
`LEADER_BASE_LINK_POSE_METRICS_VALIDATION_GUIDE.md`에도 기록되어 있다. 이번 통합은
bridge 구현을 바꾸지 않았다. 이는 종료 로그의 알려진 결함이며, 운영 중 crash 또는
외부 Terminal 1/3 종료를 뜻하지 않는다. 전체 shutdown isolation은 아래 실물 시험에서
별도로 판정한다.

## 10. 공식 2-terminal 실행

Terminal 1 — D435, VSLAM, EKF, nvblox, RViz:

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
```

Terminal 2 — Integrated Survivor pipeline (Docker YOLO + survivor nodes + image view):

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rescue_robot_bringup survivor_pipeline.launch.py
```

Raw marker가 필요 없으면 `enable_raw_visualizer:=false`, debug GUI가 필요 없으면
`show_image_view:=false`를 붙인다. 통합 launch 실행 중에는
`./scripts/run_survivor_detector.sh`를 별도로 실행하지 않는다. 이 script는
수동 fallback/debug 용도로 유지된다.

## 11. Manual Validation Quick Reference

Terminal 1을 시작하고 D435와 mapping이 준비되면 Terminal 2를 시작한다. 통합 launch
후 `ros2 node list`에서 preprocessing, detector, transform, 두 visualizer, Registry와
Registry visualizer가 한 번씩 보여야 한다. 검사용 terminal도 같은 ROS 환경을
source한다.

읽기 전용 사전 점검에서 `maze-desktop`의 USB 목록에 D435가 있었고
`rs-enumerate-devices -s`도 `RealSense D435`를 반환했다. 두 Docker image
(`damgc-vslam-mapping:humble`, `damgc-survivor-yolo:humble`)가 존재하고
`DISPLAY=:0`, `/dev/i2c-7`도 확인했다. 점검 당시 Docker container와 ROS robot
node는 실행 중이지 않았다. 이 준비 상태가 세 terminal의 성공이나 안전한 주행을
보장하지는 않는다.

```bash
ros2 node list | sort
ros2 topic info -v /leader/camera/color/image_rect
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic echo --once /leader/survivor/camera_positions
ros2 topic info -v /leader/survivor/map_positions
ros2 topic echo --once /leader/survivor/map_positions
ros2 topic info -v /leader/survivor/map_markers
ros2 topic info -v /leader/survivor/tracks
ros2 topic echo /leader/survivor/tracks --qos-durability transient_local
ros2 topic info -v /leader/survivor/registry_markers
```

사람이 없는 순간에는 `echo --once`가 기다릴 수 있다. Raw map marker는 같은
원본 timestamp의 `map_positions`와 비교한다. Registry marker는 ID별
`tracks.filtered_position`과 비교한다. Registry publish 시각과 필터링된 좌표를
raw detection의 원본 stamp/좌표와 동일하다고 가정하지 않는다. Registry reset은
현재 ID를 지우므로, 필요한 테스트 직전에만 호출한다.

```bash
ros2 service call /leader/survivor/registry/reset std_srvs/srv/Trigger "{}"
```

Reset 뒤 `tracks`가 비고 기존 registry marker가 제거되는지 확인한다. 이어서
서로 다른 위치의 두 사람 distinct ID, 이동 중 same ID, FOV 이탈→LOST,
노란 sphere와 흰 LAST SEEN text, 마지막 map 위치 및 text Z+1.0 m, 근처 재등장 시
same-ID/VISIBLE 복귀를 확인한다. 이 동작은 이전 Stage 6.1 검증과 비교한다.

VSLAM 회귀 검사는 다음과 같다.

```bash
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /leader/odometry/local
ros2 topic hz /leader/odometry/global
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info -v /nvblox_node/mesh
ros2 service list | rg /nvblox_node/get_esdf_and_gradient
```

nvblox는 mesh, 3D ESDF service, RViz를 본다. 현재 3D ESDF mode에서
`/static_esdf_pointcloud` 부재만으로 FAIL이라고 판정하지 않는다. 사람이 있는 공간과
로봇 주행은 사용자가 안전을 확인한 상태에서 수행한다. 자동 모터 명령은 없다.

## 12. 2026-09-23 실제 3-terminal 정지 상태 시험

`maze-desktop`에서 D435가 연결된 상태로 세 terminal을 순서대로 실행했다. 로봇에
이동/그리퍼 명령은 보내지 않았다. Mapping run ID는 `vslam_mapping_20260923_192138`이다.
시작 전 `/leader/camera` node는 하나였고 RGB·aligned depth는 각각 publisher 하나였다.
통합 launch 후에도 D435 node는 하나였으며 preprocessing, map transform, raw
visualizer, Registry, Registry visualizer는 각각 한 번씩 나타났다. Detector 시작 전
`/leader/person_detector`는 없었고, Terminal 3 시작 후 하나가 나타났다.

`/leader/camera/color/image_rect`에서 실제 `camera_color_optical_frame` stamp를 받았다.
Detector는 `yolo11n.pt`를 GPU device 0에서 시작했고 `camera_positions` publisher
하나와 map transform subscriber 하나가 연결되었다. `map_positions`는 transform
publisher 하나와 raw visualizer·Registry subscriber 각 하나에 연결되었다.
`map_markers`, `tracks`, `registry_markers`는 각각 publisher 하나였고 두 marker
topic은 RViz subscriber와 연결되었다. `tracks`는 `TRANSIENT_LOCAL` QoS였다.

이번 시야에서 `camera_positions.poses`는 `[]`였다. Map transform은 빈 입력을
의도대로 건너뛰므로 `map_positions`의 실제 message는 12초 `echo --once` 관찰 중
없었다. 이 시험으로 사람 Camera XYZ, Map XYZ, persistent ID, LOST, 재연결, marker
내용을 PASS로 판정하지 않는다. 사람을 카메라 앞에 세우거나 로봇을 자동으로 움직이지
않았다.

동시 실행 중 `/visual_slam/tracking/odometry` 약 5.6 Hz, local/global EKF 약
26~28 Hz를 관찰했고 `map→base_link` TF도 연속 조회됐다. nvblox mesh publisher
하나와 RViz subscriber 하나, `/nvblox_node/get_esdf_and_gradient` service를 확인했다.
mapping 종료 후 생성된 [rosbag 분석](../data/vslam_mapping_20260923_192138/analysis.md)은
192.4초 정지 구간에서 VSLAM 1,067 sample·tracking success 1,067/1,067,
local/global 5,616/5,493 sample을 기록했다. 이 수치는 정지 상태 동시 실행의 근거이며
이동 중 정밀도나 통합 전후 성능 동등성을 증명하지 않는다. Mesh 내용·3D ESDF service
응답과 실제 RViz 화면은 별도로 판정하지 않았다.

Rectify에서 RGB/CameraInfo 동기화 경고가 반복됐으나 그 로그의 10초 window에는
2~10개의 synchronized pair가 있었고 `image_rect` message도 실제 수신했다.
Detector는 RGB/aligned depth timestamp 차이가 0.133~0.267초인 sample에 대해
`N/A`를 사용했다는 경고를 냈다. 이 경고는 새 launch의 parameter 변경으로
생긴 것으로 단정하지 않으며, 사람 관측 전에는 Camera XYZ 품질을 판정하지 않는다.
`vslam_nvblox.log`에는 dual EKF의 `Failed to meet update rate` 경고 50건이
있었다. 토픽은 계속 발행됐지만 이 경고만으로 성능 회귀 원인을 결정할 수 없다.

## 13. Shutdown isolation, 판정표와 알려진 제한

실제로 Terminal 2 Ctrl+C 후 Terminal 1/3 node가 유지됐다. Terminal 2를 재시작하면
여섯 process가 다시 나타났고 `image_rect` publisher↔detector subscriber,
`camera_positions` publisher↔map transform subscriber가 재연결됐다.
Terminal 3 Ctrl+C 후 Terminal 1/2 node가 유지됐다. Terminal 1 Ctrl+C 후
mapping launcher가 rosbag을 마감·분석하고 소유 process/container를 정리했으며
Terminal 2의 node만 남았다. 마지막으로 Terminal 2를 종료한 뒤 ROS node와 Docker
container는 남지 않았다. 권장 정상 종료 순서는 detector → survivor pipeline →
mapping이다. 종료 중 bridge의 기존 이중 shutdown traceback은 재현됐지만
외부 terminal을 종료시키지 않았다.
Mapping 종료 로그의 STM32 bridge도 shutdown 중 invalid ROS publisher context로
exit code 1을 남겼다. Mapping launcher는 마감됐고 process는 남지 않았다.
이 예외는 별도 STM32 bridge 종료 처리 이슈로 기록하며 survivor launch가
STM32를 소유하거나 재시작하지는 않는다.

| 항목 | 이번 작업 결과 |
| --- | --- |
| Build / selected tests / installed launch | PASS / 121 passed, 0 failed / PASS |
| Terminal 2 단독 startup, raw on/off | PASS / PASS |
| Terminal 2 단독 중복 D435·Registry·detector | 새 D435·detector 없음, Registry 1개 — PASS |
| Camera XYZ / Map XYZ / persistent ID | `camera_positions`는 빈 배열; 실제 사람 좌표·ID 검증 NOT RUN |
| LOST / same-ID reassociation / reset | 3-terminal 사람 관측 기반 검증 NOT RUN |
| Terminal 1/3과의 shutdown isolation | 정지 상태에서 PASS |
| 3-terminal VSLAM / EKF | 정지 상태 topic·TF·rosbag PASS, 이동/성능 회귀 NOT RUN |
| 3-terminal nvblox | mesh endpoint·ESDF service 존재 PASS, 내용/RViz 판정 NOT RUN |

이전 Stage 6.1의 실물 PASS는 이번 새 launch의 실물 PASS를 대신하지 않는다.
현재 남은 제한은 3-terminal 운영, 별도 Docker lifecycle, 수동 debug GUI와 위
bridge 종료 traceback이다. Future detector integration은 반복적인 실물 세션에서
이 구조가 안정적일 때만 검토한다. 그 전에 Docker의 signal forwarding, TTY,
GPU runtime, 환경 변수, mount 및 container cleanup을 검증해야 한다.

## 14. 발생 오류·원인·해결과 rollback

개발 중 새 pytest가 처음 두 번 실패했다. 원인은 ROS Humble
`PythonLaunchDescriptionSource.location`이 child description을 load하기 전에는
실제 path가 아닌 substitution 문자열을 반환하는 점이었다. 테스트가 child를 먼저
load한 뒤 resolved path를 확인하도록 수정하여 통과했다. Runtime 종료의 bridge
traceback은 위의 기존 결함이며 이번 변경으로 해결했다고 주장하지 않는다.

새 통합 launch에 문제가 생기면 Terminal 2를 중지하고 이전 T2/T4/T5/T6 child
launch를 개별 실행한다. Detector와 mapping 실행기는 기존 명령 그대로 사용한다.
코드 rollback 시 `git status`/`git diff`로 변경 파일을 확인하고 이번에 추가한
launch·test·문서 및 bringup dependency/test 등록만 선택적으로 되돌린다.
작업 전부터 있던 사용자 문서는 보존한다. commit/push는 수행하지 않았다.
