# 실제 리더 경로 기반 곡선 운반 — 코드 인계

2026-10-27. 기존 직선 모드와 별도로 선택하는 실험 모드다. **리더 후진으로 당기기 → 실제 wheel odometry 경로 기록 → 힌지 구조를 반영한 팔로워 경로 → 진행 차이 감속 보정**을 구현했다. 고정 시간 지연은 넣지 않는다. 사용자가 확인한 힌지 범위는 정면0° 기준 좌우 각각60°로 설정했다.

## 빠른 실행

먼저 기존 직선 수동 이동 터미널을 종료한다. 서로 다른 모드의 명령 publisher를 동시에 실행하지 않는다. 기존 모드가 남으면 새 모드가 활성화를 거부한다.

팔로워(서브):

```bash
~/damgc_robot_mission/scripts/run_follower_curve.sh
```

리더(기존 mapping/selector/STM32/Dynamixel 실행 상태 재사용):

```bash
~/damgc_robot/scripts/manual_curve_terminal.sh
```

리더 저장소의 기존 `manual_straight_terminal.sh` 설정을 따라 odom은 `/leader/odom/raw`, 집기값은477, settle9초를 사용한다. 필요하면 `LEADER_ODOM_TOPIC`, `LEADER_CLOSE_RAW` 환경변수로 조정한다. 팔로워는 집기331·열림1000·speed40, 리프트 제외다.

| 키 | 곡선 모드 동작 |
|---|---|
| e | 서브 준비 후 리더 활성화 |
| ↑ / w 유지 | **리더가 후진으로 당김**, 팔로워가 앞으로 추종 |
| 이동 중 ← / a 유지 | 왼쪽 완만한 곡선 |
| 이동 중 → / d 유지 | 오른쪽 완만한 곡선 |
| ↓ / s / x 또는 이동키 해제 | 감속·정지 |
| G / R | 기존 양쪽 파지·열기. 동작 중 주행 잠금 |
| Space / q | 베이스 정지·gate 해제 / 종료 |

곡선 모드의 ↑는 리더 후진이다. 기존 앞뒤 직선 모드와 구분한다. 제자리 회전과 진행 방향 반전은 이 버전에 넣지 않았다. 방향을 바꿔 조립하거나 준비 뒤 위치를 바꾸면 새 세션으로 다시 준비한다.

## 코드가 하는 일

- 리더의 실제 pose를5mm 간격, 최대512개 rolling trail로 보관한다. 모터 명령을 경로처럼 가정하지 않는다.
- 리더 차축→힌지0.125m, 힌지→상자 접촉0.1325m, 상자 중심→접촉0.0525m 기본 모델을 재사용한다. 중립 배치의 차축 간격은0.62m다. 물품·팔 자세가 다르면 기하값도 맞춰야 한다.
- 실제 리더 힌지의 이동에서 팔로워의 전후 진행량과 회전을 적분한다. 곡률이 변할 때 고정 곡률 평형 자세만 사용하지 않는다. 측방 미끄러짐을 만들지 않는 차동구동 적분과 힌지 사이 길이 제약을 사용한다.
- 서브는 전달된 실제 경로와 rolling 기준값을 사용해 자기 경로를 다시 계산한다. 두 로봇의 odom 원점이 달라도 중립·마주 보는 초기 배치를 기준으로 SE(2) 변환한다. frame 이름이 같다는 이유로 좌표를 공유하지 않는다.
- 서브는 경로 진행 차이·가로 오차를 리더에 돌려준다. 오차가 커지면 리더 속도를 부드럽게 줄이고 서브는 feedforward와 위치·방향 보정을 계속한다.
- **경로 오차 임계값 초과 FAULT는 없다.** ordinary tracking error로 세션을 종료하지 않는다. 명령과 힌지 곡률은 포화 제한한다.
- 리더 명령0이면 서브는 경로 끝까지 계속 가지 않고0속도를 낸다. 리더 실제 속도가0이면 서브도 움직이지 않는다.
- 입력·통신·ACK·odometry·경로 데이터 유실, pose reset/frame 변경 등 데이터 이상, Space 정지는 유지한다. 끊김 뒤 자동 재출발하지 않는다.

motion은 기본 false이며, 별도 실행 스크립트가 명시적으로 켠다. 최대 리더 속도0.02m/s·가속도0.02m/s², 리더 요청 곡률은최대1.0m⁻¹, 곡률 변화는초당0.25m⁻¹로 제한한다. 서브 보정 출력은최대0.04m/s·0.08rad/s이며 최종 구동 guard도 거친다. 힌지60°에20% 여유를 둔 기하를 사용한다.

## 개별 실행·상태

```bash
ros2 launch cooperative_manual manual_curve.launch.py role:=follower motion_enabled:=true \
  gripper_enabled:=true close_raw:=331
ros2 service call /follower/manual_curve/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /cooperation/curve/follower/status
ros2 topic echo /cooperation/curve/leader/status
```

- 서비스: `/{leader,follower}/manual_curve/enable` (`SetBool`).
- 전용 입력: `/cooperation/curve/input` (`Twist`), 리더 역방향 linear.x와 angular.z.
- session/seq/ACK: `/cooperation/curve/{leader,follower}`.
- 실제 리더 trail: `/cooperation/curve/trail` (`String` JSON, bounded queue1).
- RViz 경로: `/cooperation/curve/{leader,follower}/path` (`nav_msgs/Path`, 각 로컬 odom frame).
- G/R은 기존 `/cooperation/manual/gripper/*` 통신과 모터 노드를 재사용한다.

## 검증 범위

좌·우 곡선의 이상적인 unicycle/odometry 시뮬레이션, rolling buffer, 서로 다른 odom 원점, 큰 오차의 비중단 보정, 데이터 유실 정지, straight 회귀를 확인했다. 별도 localhost ROS domain176에서는 가짜 selector/guard·합성 odom으로 양쪽 READY, 곡선 명령, 진행 오차 감속, 경로 데이터 단절 시 정지를 확인했다. 실제 로봇이나 모터를 곡선 모드로 구동하지 않았다.

이것은 이상적인 기구학 모델이다. 실제 바퀴 미끄러짐, 힌지 마찰, 물체 힘·하중, 실제 출발 지연은 검증하지 않았다. 먼저 상자 없이 완만한 곡선에서 방향과 명령을 확인하고, 이후 합체 시험에서 기하·바퀴 보정을 확인해야 한다. 경로 오차 하드 스톱을 제거했으므로 큰 오차는 감속·보정이 지속되며 자동으로 실패 판정해 종료하지 않는다.

검증 결과: 팔로워 작업공간의 직선·곡선·기존 운반 회귀73개 통과. 리더 최신 main `2cd7a97`을 기준으로 만든 별도 작업공간에서도 곡선 수학·ROS 연결11개 통과 및 두 패키지 빌드 성공. 모든 ROS 모의 구동은 localhost 전용 domain176에서 수행했다.

## 배포

리더 쪽은 최신 `rbgusrns/damgc_robot` main을 기반으로 별도 `codex/manual-curve-20261027` 브랜치에 작업했다. 기존 직선 키보드·매핑·임무 코드는 유지한다. 팔로워 쪽 `kmhuh1525/damgc_robot`의 `codex/cooperative-mission-20260930`에도 동일한 곡선 controller/math를 포함한다.

각 작업공간에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cooperative_transport cooperative_manual --symlink-install
```


### 저속 튐과 터미널 조향 보정
서브는 리더 실측 속도를 0.25초 저역통과 필터로 평활화하고 최종 선속도 가속도 0.02 m/s², 각가속도 0.04 rad/s²로 제한한다. 명시적 정지는 즉시 0이다. 터미널 동시 키 입력 대신 ↑/W 하나를 유지하면 직진, ←/A 하나는 좌곡선, →/D 하나는 우곡선이며 키를 놓으면 정지한다. 리더 키보드 노드도 새 코드로 재시작해야 한다.

↓/S를 유지하면 반대 방향으로 직선 복귀한다(리더 전진, 서브 후진). X=정지, Space=비상정지. 전후진 전환은 기존 속도 램프로 0을 통과한다. 곡선 후진 입력은 허용하지 않는다. 리더/서브 모두 이 버전으로 업데이트해야 한다. 실기 재검증 전이며 양쪽 서비스는 명시적으로 재활성화해야 한다.


### 정지 부근 미세 구동 억제 (코드 변경, 실행 중인 노드 미재시작)
수동 입력과 무관하게 실측 속도 잡음과 작은 위치 오차가 서브 저속 펄스를 만들 수 있다. 서브 속도 필터에 시작 1.5 mm/s / 정지 0.7 mm/s 히스테리시스, 위치 보정 4 mm·각도 0.01 rad 데드존, 추격 속도 실측 기준 1.5배 제한을 추가했다. 최종 선가속도 최대 0.012 m/s², 가속도 변화율 최대 0.04 m/s³로 부드럽게 제한한다. 리더의 추종 오차 기반 감속 비율도 완만하게 변화한다. 비상정지와 명시적 0 명령은 즉시 0이다. 저속 PID/마찰까지 실물에서 해결됐다고 보장하지 않으며 재시작 후 실기 확인이 필요하다. Nav2 단독 주행에는 이 곡선 추종 모듈이 자동 적용되지 않는다.
