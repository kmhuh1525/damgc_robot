# nvblox 기반 Nav2 목표 확인

현재 구성은 VSLAM이 `odom -> base_link`를 발행하고 nvblox가 `odom` 프레임의
`/nvblox_node/static_map_slice`를 발행한다. Nav2 global/local costmap 모두 이 slice를
사용한다. Nav2 자체는 `map` 프레임이나 AMCL을 요구하지 않는다. Survivor의
장기 위치 등록에는 `map`이 필요하므로 VSLAM이 `map -> odom`도 발행한다.

카메라가 이미 실행 중일 때 컨테이너에서 다음 launch를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install_docker/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 DAMGC_VSLAM_HEADLESS=1
ros2 pkg prefix rescue_robot_bringup
ros2 launch rescue_robot_bringup nvblox_vslam_realsense.launch.py
```

`ros2 pkg prefix` 결과가 `/workspaces/isaac_ros-dev/install_docker/rescue_robot_bringup`인지
확인한다. 호스트의 `/home/maze/damgc_robot/install`이 나오면 컨테이너 안의 새 터미널에서
위 두 setup 파일을 순서대로 다시 source한다. 새 launch가 `install_docker`에 없다면
컨테이너에서 다음 명령으로 설치한다.

```bash
cd /workspaces/isaac_ros-dev
colcon --log-base log_docker build --packages-select rescue_robot_bringup \
  --symlink-install --build-base build_docker --install-base install_docker
source install_docker/local_setup.bash
```

기존 `nvblox_costmap.launch.py` 또는 다른 VSLAM/nvblox launch와 동시에 실행하지 않는다.
카메라·VSLAM·nvblox가 이미 실행 중이면 `nvblox_nav2.launch.py`만 실행할 수 있다.
특히 `visual_slam_nvblox_realsense.launch.py`는 dual EKF가 TF를 발행하므로
VSLAM의 `publish_odom_to_base_tf` 값이 `false`다. 위 전용 launch에서는
`publish_odom_to_base_tf`와 `publish_map_to_odom_tf`가 모두 `true`다.
따라서 이 launch와 dual EKF launch를 동시에 실행하면 TF publisher가 중복된다.

별도 컨테이너 터미널에서 같은 setup 파일을 source한 뒤
`rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz`를 실행한다.
RViz의 Fixed Frame은 `odom`이고
`Nav2 Goal` 도구가 있다. 이 도구는 `/navigate_to_pose` action으로 목표를 보낸다.
계산된 경로는 주황색 `Nav2 Plan` 표시(`/plan`)에서 확인한다.
로봇 가까이, 현재 확인된 자유 공간에 작은 목표를 지정한다. `/nav2/cmd_vel`은
바퀴 브리지의 `/leader/cmd_vel`과 연결되지 않아 이 구성만으로 바퀴는 움직이지 않는다.
`Nav2 Plan`은 메시 위에 그리도록 RViz에서 Z 오프셋 0.2 m로 설정했다.
RViz를 이미 실행 중이었다면 변경된 `vslam_nvblox.rviz` 파일로 다시 실행한다.

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list -t | grep navigate_to_pose
ros2 topic info /nvblox_node/static_map_slice
ros2 topic info /plan
ros2 topic info /nav2/cmd_vel
```

`Nav2 Goal`로 목표를 지정하기 전에 다른 터미널에서
`ros2 topic echo --once /plan --field header.frame_id`를 실행하면
계획 성공 시 `odom`이 출력된다. 목표는 global costmap의 알려진 자유 공간에
설정해야 하며, 경로가 없으면 planner 로그와 목표 좌표를 확인한다.

활성 상태는 각각 `active [3]`이어야 하고 slice 구독자는 두 costmap이다.
`/nav2/cmd_vel`에는 controller publisher 하나와 subscriber 0개가 있어야 한다.

로컬 costmap은 `/local_costmap/costmap`에서 발행하고, 호환 토픽
`/costmap/costmap`에도 같은 `nav_msgs/msg/OccupancyGrid`를 발행한다.
두 토픽 모두 `header.frame_id`가 `odom`이며, 호환 토픽은 전체 `data` 배열을
2 Hz로 제공한다.
통합 launch를 재시작한 뒤 다음 명령으로 확인한다.

```bash
ros2 topic echo --once /costmap/costmap
ros2 topic hz /costmap/costmap
```

## Survivor와 함께 실행할 때: TF 및 depth 동기화

2026-09-26의 Nav2 통합(`1b201a8`)에서 전용 VSLAM launch의
`publish_map_to_odom_tf`가 `true`에서 `false`로 바뀌었다. 전용 실행 경로에는
global EKF가 없어 `map` 프레임 자체가 사라졌고, Survivor Stage 4의 영상 시각
`camera -> map` 조회가 실패했다. Stage 5 marker와 Stage 6 registry도 따라서
비어 있었다. 전용 VSLAM launch에서 `map -> odom`을 다시 켰으며 소유권은
`map -> odom`: VSLAM, `odom -> base_link`: VSLAM,
`base_link -> camera_link`: robot_state_publisher의 고정 관절,
`camera_link -> optical frame`: RealSense이다. Nav2와 nvblox의 `odom` 설정 및
RViz Fixed Frame은 유지한다. 다른 VSLAM/EKF launch를 동시에 실행하지 않는다.
VSLAM TF는 영상 처리 시점에 간헐적으로 늦게 도착한다. 실측 328개 sample에서
영상 stamp 대비 `odom` TF 지연은 95백분위 133 ms, 99백분위 300 ms,
최대 467 ms였다. Stage 4는 같은 영상 stamp의 TF를 최대 0.6초 기다린다.
lookup timestamp를 0으로 바꾸거나 `odom` 좌표를 `map`으로 간주하지 않는다.

또한 YOLO가 단일 스레드 RGB callback에서 실행되는 동안 aligned depth callback이
지연될 수 있었다. D435를 매핑 스크립트와 같은 인자로 18초 실행해 원본 RGB 478개와
aligned depth 490개를 비교한 결과 최근접 header stamp 차이는 중앙값과 95백분위
모두 0 ms, 최대 33.36 ms, 120 ms 초과 0건이었다. 따라서 이 장비의 해당 실행에서
관찰된 133~167 ms 경고의 원인은 카메라 프로파일보다 detector의 callback/cache
경로다. 실제 로그에는 depth가 RGB보다 133 ms 오래된 경우와 RGB 처리가
589 ms 늦어 depth보다 400 ms 오래된 경우가 모두 있었다. detector의 ROS
callback은 최신 RGB 한 장과 depth cache만 갱신하고, 별도 worker가 최대 150 ms
동안 더 가까운 depth를 기다린 뒤 YOLO를 실행한다. stamp 기준 0.5초와
`sync_queue_size`로 depth cache를 제한하며 기존 `sync_slop_sec=0.12`는 유지한다.
callback에 250 ms 이상 늦게 도착한 RGB는 추론 전에 버려 밀린 영상을 새 depth와
잘못 짝짓지 않는다.
카메라 자동 선택 프로파일
color `640x480x30`, depth/infra `848x480x30`, `enable_sync` 및 depth alignment도
유지한다.

변경 파일은 `launch/nvblox_vslam_realsense.launch.py`, Survivor detector,
Stage 4 launch와 노드의 TF 대기 기본값, 관련 회귀 테스트 및 이 문서와 Survivor
설명 문서다. `run_vslam_mapping.sh`, AprilTag, STM32, Nav2 설정,
Survivor Stage 4의 exact-time 조회 정책은 그대로다.

빌드 및 실행 순서:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_bringup rescue_robot_survivor
source install/local_setup.bash
# detector 코드가 이미지에 포함되므로 해당 이미지는 다시 빌드
./scripts/build_survivor_runtime.sh
```

매핑 컨테이너의 기존 `install_docker`가 저장소 launch를 symlink로 참조하는지
확인한다. 설치본이 오래된 경우 매핑 컨테이너를 띄운 뒤 위 문서 앞부분의
`colcon --build-base build_docker --install-base install_docker` 명령으로
`rescue_robot_bringup`만 다시 빌드하고 매핑 launch를 재시작한다.
Terminal 1에서 `./scripts/run_vslam_mapping.sh`, Terminal 2에서 ROS setup과
`install/local_setup.bash`를 source하고
`ros2 launch rescue_robot_bringup survivor_pipeline.launch.py`, Terminal 3에서
`./scripts/run_survivor_detector.sh`를 실행한다.

Terminal 4 검증:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 node list | grep -E 'visual_slam|ekf|nvblox|planner|controller|bt_navigator'
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo --once /leader/survivor/camera_positions --field header.frame_id
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
ros2 topic echo --once /leader/survivor/camera_positions
ros2 topic echo --once /leader/survivor/map_positions
ros2 topic echo --once /leader/survivor/tracks --qos-durability transient_local
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic info /nvblox_node/mesh
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_tools view_frames
```

`camera_positions`의 실제 `frame_id`가 위 예시와 다르면 그 값으로 `tf2_echo`를
실행한다. 사람이 보이는 상태에서 camera/map positions에 유효한 XYZ, raw marker,
충분한 관측 뒤 tracks에 ID가 있어야 한다. RViz Nav2 Goal로 경로와 `/plan`의
`odom` frame을 확인한다. `/tf`·`/tf_static`의 publisher와 생성된 TF tree에서
두 dynamic TF의 중복이나 cycle도 확인한다. Stage 4는 정확한 영상 timestamp의
TF만 사용하므로 `latest` 조회로 우회하지 않는다.

관련 자동 회귀 테스트는 `colcon test --packages-select rescue_robot_bringup
rescue_robot_survivor`와 `colcon test-result --verbose`로 실행한다.

2026-10-02 현장 검증: 두 변경 패키지 빌드와 Python syntax 검사가 성공했다.
Survivor 전체 테스트 및 bringup Survivor launch 테스트 125개가 통과했다. 두 패키지의
전체 pytest 결과는 128 passed, 2 failed였으며 두 실패는 이번 변경 전부터
존재한 AprilTag launch 테스트의 이전 그리퍼 닫힘값 `450`과 접근 거리 `0.16`
기대값이다. 현재 main의 실제 기본값은 각각 다른 값이므로 이 작업에서
AprilTag 파일이나 해당 테스트는 변경하지 않았다. 실장비에서는 `map -> odom`,
`odom -> base_link`, `map -> camera_color_optical_frame`이 지속 출력됐고,
`view_frames`에 하나의 `map -> odom -> base_link -> camera_link` 경로가 있었다.
nvblox mesh는 12초에 76건(최대 11,390 vertices)이 발행됐고 Nav2
`ComputePathToPose`는 `odom` 경로를 반환하며 성공했다. 사람 관측 20초에
camera_positions 163건과 map_positions 150건이 non-empty였으며 raw/registry
marker도 발행되고 registry에 Survivor ID가 나타났다. detector의 마지막
5분 로그에는 RGB/depth mismatch 경고가 없었다. 0.6초 TF 대기 설정으로
워밍업 이후 실제 영상 stamp 240회 중 exact-time TF 조회 실패는 0건이었다.
`NavigateToPose` 목표는 수락됐으나 현장 costmap에서 controller가 유효한
trajectory를 찾지 못해 중단됐다. 따라서 주행 완료는 확인되지 않았으며
Nav2 경로 생성과 노드 활성 상태까지만 검증됐다.
로봇을 이동시키며 map 좌표 안정성을 확인하는 시험은 별도로 필요하다.
