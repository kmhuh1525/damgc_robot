# Follower 협동 운반 준비 상태 (2026-10-04)

최신 계산식 변경은 [Leader d812a04 동기화 기록](FOLLOWER_LEADER_D812A04_SYNC.md)을 따른다.
아래 초기 준비 검증 기록 이후 Leader의 직접 곡률 변환식도 적용했다.
이후 사용자 요청으로 실차 실행의 이동·I2C 쓰기를 true로 전환했다.
실제 실행 결과와 최종 상태는 [최종 진행 기록](FOLLOWER_COOPERATIVE_TRANSPORT_PROGRESS_2026-10-04.md)을 따른다.

## 완료한 작업

새 `cooperative_transport` 패키지와 Follower selector·guard·STM32 bridge를 빌드했다.
새 패키지의 기본값은 `motion_enabled=false`, `i2c_write_enabled=false`다.
준비와 경로 표시는 가능하지만 START/ARM은 이동 허용을 명시하기 전까지 거부된다.
selector/guard 서비스 무응답은 2초 뒤 정지 처리하고, 잘못된 JSON 최상위 타입과
준비 검증 실패가 실행 상태로 이어지지 않도록 보완했다.

ROS domain 173, localhost 전용에서 가상 Leader/odometry와 실제 ROS selector·guard를
연결해 15개 테스트가 통과했다. STM32 bridge, I2C, U2D2 또는 실제 모터는 테스트에 연결하지 않았다.
테스트가 만든 준비·예약 시작 상태는 이 격리 환경에서만 사용했다.

- 준비 중 최종 출력 0, selector STOP, 로컬 odom frame으로 경로 변환
- 기본 이동 gate의 ARM 거부, 가상 예약 시작 후 후진 출력과 STOP
- READY 및 가상 RUNNING 상태의 heartbeat/odometry timeout 정지
- 해시 변조, 기하 불일치, 독립 계산과 다른 경로, 중복 publisher 거부
- 늦은 COMMIT 거부, 서비스 무응답 timeout, 비정상 JSON 처리
- 실제 설치된 launch의 IDLE, motion disabled, guard disabled, reverse 허용 설정
- command → selector → guard publisher/subscriber 연결 및 단일 소유권

실제 Leader 경로·두 Orin DDS 통합·상자 결합 주행·예약 시작 시차는 아직 확인하지 않았다.
이 작업이 끝난 상태에서는 실제 도메인에 협동 launch를 대기시키거나 PREPARE/START를
보내지 않는다. 격리 테스트 프로세스도 종료한다.

## 다음 연결 시험용 실행

이 명령은 실제 연결 시험을 시작할 때 사용한다. 기존 mission/selector/guard/bridge와
새 launch를 중복 실행하지 않는다. 아래 domain/RMW는 새 Leader README 예시와 같다.
실제 Leader가 다른 domain/RMW를 선택했다면 양쪽을 함께 맞춘다.

```bash
cd /home/kde/damgc_robot_mission
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch cooperative_transport manual_transport.launch.py \
  role:=follower follower_drive:=true use_stm32_bridge:=true \
  motion_enabled:=false i2c_write_enabled:=false
```

bridge는 odometry를 읽고 속도 쓰기는 하지 않는다. 물체를 수동으로 잡아 중립 힌지·
반대 방향·차축 사이 62 cm를 맞춘 상태에서 Leader B → 새 RViz goal → READY까지만
확인한다. 상대 조립 자세는 센서로 계측하지 않고 이 물리 기준을 가정한다.
READY 후 움직이거나 조립을 바꾸면 새 준비와 경로가 필요하다.

나중에 구동 시험을 진행할 때 Follower launch에 `motion_enabled:=true`와
`i2c_write_enabled:=true`를 모두 명시하고 Leader peer도 `motion_enabled:=true`로
시작해야 한다. 이번 작업에서는 이 실차 실행을 하지 않았다.

## Leader 연결에서 남은 항목

- 현재 공용 Leader selector에 peer가 요구하는 COOPERATION source와
  `enable_nav2_goal_selection` 연결을 반영해야 한다.
- `transport_keys`의 `/leader/mapping/control` 요청을
  `/cooperation/transport/control`의 PREPARE/START/ABORT로 변환하는 중계가 필요하다.
- Leader의 `/plan`, FollowPath, odometry, costmap을 실제 실행 환경에서 확인해야 한다.
  현재 Follower 호스트에는 `nav2_msgs`가 없어 Leader 역할의 peer를 실행하지 않았다.
  Follower 역할은 FollowPath를 import하지 않아 실행 가능하다.
- 기존 [요구사항 대조](FOLLOWER_LEADER_REQUEST_AUDIT_2026-10-04.md)의 untracked 표시는
  최초 조사 시점의 기록이다. 이번 준비 변경에서는 새 패키지도 코드 공유 대상에 포함했다.

## 검증 명령

추가 임시 변경: 곡률 검증 생략은 양쪽 peer에 `validate_curvature:=false`를 명시한다.
곡률·힌지 각도 한계 검사만 생략하고, 경로 계산과 횡이동·round-trip·정렬·해시 검사는
유지한다. 기본값은 true다. 현재 실기 Follower는 이 옵션을 false로 재기동하며
`motion_enabled=false`, `i2c_write_enabled=false`를 유지한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select \
  cooperative_transport follower_command_selector follower_control stm32_bridge
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest -q src/cooperative_transport/test
```
