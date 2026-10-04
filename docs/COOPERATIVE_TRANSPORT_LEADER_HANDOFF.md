# 협동 운반: Leader 경로 전달 인계 메모

작성일: 2026-10-04
상태: Leader Nav2 경로 handoff 구현과 인터페이스를 맞추기 위한 초안

## 목표 흐름

1. 작업자가 두 로봇을 상자를 사이에 두고 반대 방향으로 마주 보게 조립한다. 파지와 리프트는 이 시험에서 수동으로 준비한다.
2. 작업자가 협동 준비를 요청한다. Leader는 주행 명령을 막고, 이전 계획을 버린 뒤 새 Nav2 목표를 기다린다.
3. 새 목표로부터 생성된 `/plan`을 Leader가 수신한다. Leader는 frame, timestamp, 시작 pose, 간격, 힌지 곡률, 양쪽 차축 경로, 장애물 여유를 검증한다.
4. Leader는 검증한 동일한 경로 데이터와 session/hash를 Follower에 보낸다. Follower는 형상과 해시를 독립 검증하고, 자기 `/follower/odom/raw` frame으로 변환한 경로의 시작 정렬을 확인한다.
5. 양쪽 로봇이 정지한 상태로 `READY`를 확인한다. 작업자의 명시적 START 뒤에만 selector/guard를 준비하고, 두 로봇이 경로를 추종한다.

경로 수신, 계산, `READY`만으로 로봇을 움직이면 안 된다. 이 단계는 양쪽 로봇이 상자를 수동으로 잡은 상태에서 제어 흐름을 검증하기 위한 것이며, 자동 grasp/lift는 범위에 없다.

## 현재 제안된 경로·상태 인터페이스

| 용도 | 토픽/액션 | 형식 및 frame |
|---|---|---|
| Leader Nav2 계획 | `/plan` | `nav_msgs/Path`; 현재 Nav2 global frame은 `odom`으로 설정됨 |
| Leader odometry | `/leader/odometry/local` | `nav_msgs/Odometry`; 계획과 같은 local `odom` 좌표계여야 함 |
| Follower odometry | `/follower/odom/raw` | `nav_msgs/Odometry`; 별도 로컬 odom 원점 사용 가능 |
| 양쪽 peer 통신 | `/cooperation/transport/leader`, `/cooperation/transport/follower` | `std_msgs/String`, versioned JSON: session, path hash, PREPARE/READY, ARM/ACK, COMMIT/ACK, heartbeat, STOP |
| RViz 경로/상태 | `/cooperation/transport/{leader,follower}/path`, `/cooperation/transport/{leader,follower}/status` | `Path`는 각 로봇의 로컬 frame; status는 transient-local |
| Leader RPP 입력·경로 실행 | `/nav2/cmd_vel`, `/follow_path` | controller 출력은 command selector와 분리. `/follow_path`는 Nav2 `FollowPath` action |
| 협동 drive 명령 | `/leader/cooperation/cmd_vel`, `/follower/mission/cmd_vel` | 각각 Leader/Follower peer의 `Twist` 출력 |
| Follower 구동 gate | `/follower/command_selector/set_parameters`, `/follower/velocity_guard/enable` | selector는 `STOP`으로 시작하고 guard는 닫힌 상태여야 함 |
| 장애물 검사 | `/global_costmap/costmap`, `/global_costmap/costmap_updates` | 신선하고 알려진 영역인 Leader global costmap |

Peer packet의 body에는 frame, 기하 파라미터, Leader/Follower 차축 pose 배열, 예상 Follower 시작 pose, 속도가 포함된다. Follower는 Leader와 같은 frame 이름을 공유한다고 가정하지 않는다. 중립 힌지 상태에서 측정 기하로 한 세션 동안 SE(2) 정렬을 계산한다. 로봇을 움직이거나 조립 상태를 바꾸면 새 계획·준비부터 다시 해야 한다.

## Leader 쪽에서 맞춰야 할 항목

현재 공용 브랜치 기준으로 아래 연결이 필요하다. Leader 구현에서 다른 인터페이스를 선택한다면 양쪽 코드를 같은 계약으로 맞춰야 한다.

1. **Command selector 소유권:** Leader peer는 기본적으로 `/leader/cooperation/cmd_vel`에 명령을 발행하고 `source_mode=COOPERATION`을 요청한다. 현재 공용 Leader selector는 `STOP/TELEOP/APPROACH/NAV2/MISSION`만 받으며 이 입력을 구독하지 않는다. selector에 새 안전 source를 추가하거나, velocity guard를 통과하는 기존 source로 경로를 바꾸고 peer·launch·문서를 함께 수정한다. selector를 우회해 motor topic에 직접 연결하지 않는다.
2. **Nav2 목표 선택 gate:** 현재 peer 초안은 `/leader/command_selector/set_parameters`에서 `enable_nav2_goal_selection=false`를 설정하려 한다. 이 파라미터는 현재 Leader selector에 선언되어 있지 않다. 실제 목표 선택을 소유하는 노드/키보드에 gate를 추가하거나 이 호출을 제거하고, 새 목표의 action status와 `/plan` timestamp로 오래된 계획이 선택되지 않도록 한다.
3. **B/N/중지 요청 전달:** 별도 `transport_keys`는 `/leader/mapping/control`에 `COOP_PREPARE`, `COOP_START`, `COOP_ABORT`를 발행한다. peer 초안의 실제 입력은 `/cooperation/transport/control`이며 값은 `PREPARE`, `START`, `ABORT`다. Leader bringup/키보드가 두 계약 사이의 요청을 변환해 전달하고, 일반 teleop·다른 임무 키가 들어오면 협동 세션을 취소해야 한다.
4. **새 Nav2 계획 판별:** PREPARE 이후 새 NavigateToPose goal을 확인하고 그 goal에 해당하는 `/plan`만 채택한다. `/navigate_to_pose/_action/status`와 plan stamp를 사용할 경우 clock domain, stamp 비교, 재계획/취소 동작을 확인한다.
5. **frame 및 시작 정렬:** `/plan.header.frame_id`는 Leader odometry frame과 일치해야 한다. 현재 초안 검사는 시작 위치 오차 5 cm, 방향 오차 3°, 인접 pose 간격 8 cm 이하, pose 3–4000개를 요구한다. Nav2 설정 변경으로 조건이 달라지면 한쪽만 완화하지 말고 Follower 검증과 함께 갱신한다.
6. **Follower-feasibility 거부:** 힌지 곡률, 차축 비측방 이동, 일정한 전진/후진 방향, 작업 공간/costmap 검사를 통과하지 못하면 계획을 폐기하고 정지 상태를 유지한다. 현재 경로 변환기는 거부 사유를 반환할 뿐 Nav2 대체 경로를 자동 탐색하지 않는다.
7. **FollowPath command 흐름:** 준비 중에는 motor selector를 닫아 둔다. START 승인 뒤 Nav2 `FollowPath`가 동결된 경로를 수행하고 RPP의 `/nav2/cmd_vel`을 관찰할 수 있어야 한다. 별도 cooperative gate는 예정된 시각 전까지 0 명령이어야 하며 RPP 정지/취소/실패 시 양쪽을 정지시킨다.
8. **장애물 frame:** Leader costmap frame이 Leader local odom과 같은지 확인한다. 초안은 알려지지 않은 셀도 안전 공간으로 취급하지 않고, 양쪽 로봇과 상자의 합친 외곽이 costmap 바깥이나 장애물에 닿으면 거부한다.

## Follower launch 충돌 주의

현재 일반 `follower_mission.launch.py`의 경로 추종 서비스는 Follower 단독 시험용이다. 새 peer workflow는 별도 selector/guard/bridge 설정을 갖는다. 두 launch를 함께 켜면 selector·guard·STM32 bridge 또는 `/follower/mission/cmd_vel` publisher가 겹칠 수 있다. 실기 연결 전 launch 하나만 소유하도록 정리하고, bridge는 한 개만 실행한다. Peer의 readiness 검사는 command topic publisher가 정확히 하나인지 확인한다.

## 무구동 인수 확인 순서

1. Leader와 Follower의 ROS domain, RMW, 네트워크 연결을 맞춘다. 두 launch 예시의 `ROS_DOMAIN_ID` 값도 같아야 한다.
2. 양쪽 peer 및 status/path 토픽이 보이는지 확인한다. Leader command selector는 `STOP`, Follower selector는 `STOP`, velocity guard는 disabled여야 한다.
3. 양쪽 `/odom` 신선도와 frame을 확인한다. 상자는 잡은 상태로 고정하고, 두 로봇은 중립·직선 힌지로 서로 반대 방향을 향하게 한다.
4. PREPARE 후 새로운 Nav2 목표 하나를 보내고, Leader가 새 `/plan`을 받았는지 확인한다. 이 단계에서 `READY`가 오기 전까지 command gate는 닫혀 있어야 한다.
5. Leader가 계획 거부 사유 또는 `READY`를 표시하는지, Follower가 독립 검증 후 `READY`를 보냈는지, 양쪽 RViz path가 각자 올바른 로컬 odom에 그려지는지 기록한다.
6. 이번 인수 단계에서는 START/N을 누르지 않는다. 실제 이동은 양쪽 정지 경로와 취소 동작을 별도로 점검한 뒤 승인한다.

## 구현 출처와 작업 상태

이 인계 메모는 공용 브랜치의 [협동 경로 추종 진행상황](COOPERATIVE_PATH_TRACKING_PROGRESS.md)과 현재 작업 트리에 있는 `src/cooperative_transport` peer 초안을 대조해 작성했다. `cooperative_transport` 디렉터리는 현재 미커밋/untracked 상태라 GitHub에서 코드를 볼 수 없다. 이 문서가 공유하는 것은 우선 Leader가 맞춰야 할 인터페이스와 인수 조건이며, 코드 병합 전 실제 topic/action/service 이름이 바뀌면 이 문서도 함께 갱신한다.

2026-10-04 준비 작업 후속 기록: 새 peer 패키지의 Follower 빌드·격리 검증과
기본 이동 차단 설정은 [Follower 준비 상태](FOLLOWER_MANUAL_TRANSPORT_PREPARATION.md)에 있다.
위 미커밋 상태 설명은 최초 인계 조사 당시의 기록이다.
