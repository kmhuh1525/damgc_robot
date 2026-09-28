# nvblox 기반 Nav2 목표 확인

현재 구성은 VSLAM이 `odom -> base_link`를 발행하고 nvblox가 `odom` 프레임의
`/nvblox_node/static_map_slice`를 발행한다. Nav2 global/local costmap 모두 이 slice를
사용한다. 별도의 `map` 프레임이나 AMCL은 필요하지 않다.

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
VSLAM의 `publish_odom_to_base_tf` 값이 `false`다. 위 전용 launch에서는 이 값이
`true`이며 `publish_map_to_odom_tf`는 `false`다.

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
