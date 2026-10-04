# 협동운반 경로 생성·변환·추종

## 구현 범위

협동운반 경로를 세 단계로 계산한다.

1. 상자 시작·목표 pose에서 장애물과 작업 경계를 고려한 상자 중심 경로를 만든다.
2. Leader의 axle 경로 또는 상자 경로에서 Leader/Follower axle 경로를 계산한다.
3. Follower 경로와 odometry에서 pure-pursuit 속도를 계산해 기존 guarded command 경로로 보낸다.

Leader `/plan` 수신과 경로 계산은 자동이다. 경로를 받았다고 로봇이 움직이지는 않으며,
실제 추종은 `/follower/path_tracking/enable` 서비스로 명시적으로 시작한다. 현재 실행 게이트는
Follower mission state `IDLE`이며 독립 단독 시험용이다.

## 사용자가 제공한 치수와 모델 가정

- 운반 상자: 가로 10.5 cm, 세로 16 cm, 높이 20 cm. 파지점 높이는 바닥에서 7 cm이며,
  운반 시 바닥에서 2 cm 들어 올린다. 높이는 현재 2D 경로 계산에는 사용하지 않는다.
- 두 로봇은 같은 형상이고 상자 양쪽에서 중앙을 파지한다. 각 `base_link`에서 상자 중심까지
  31 cm, 상자 두께는 10.5 cm다. 따라서 각 base에서 면 중앙 접촉점까지는 25.75 cm다.
- `base_link`는 바퀴축 중점으로 가정한다. URDF의 좌우 바퀴 간격은 23 cm다.
- 각 로봇과 그리퍼 사이에는 모터 없는 yaw 베어링 힌지가 있다. 각 힌지는 중립에서 좌우 15°,
  즉 `[-15°, +15°]` 회전할 수 있다고 모델링한다.
- 힌지 중심은 바퀴축 중점에서 12.5 cm 떨어져 있다. 힌지가 로봇 전방 중심선에 놓이고 링크가
  중립 자세에서 파지면과 정렬된다고 가정한다. base-to-contact 25.75 cm에서 12.5 cm를 뺀
  13.25 cm를 hinge-to-contact 거리로 두며, 상자 중심에서 접촉면까지는 5.25 cm다.
- URDF의 그리퍼 tip 원점 30.5 cm는 사용자가 확인한 실제 파지 TCP 25.75 cm와 다른 기준이다.

이 기하 모델은 `cooperative_mission/hinged_formation.py`에 있다. 실물 힌지 중심·링크 방향,
베어링 마찰과 관성은 아직 대조하지 않았다.

## 힌지 제약과 두 로봇 경로

`HingeGeometry`의 주요 길이는 다음과 같다.

| 항목 | 값 |
|---|---:|
| axle에서 hinge 중심 | 0.125 m |
| hinge에서 접촉점 | 0.1325 m |
| 상자 중심에서 접촉점 | 0.0525 m |
| 물리 힌지 범위 | ±15° |
| 계획 여유 적용 후 허용 범위 | ±12° |

상자 경로의 곡률 `k`로 준정적 힌지 각을 계산한다. `d = hinge_to_contact + object_center_to_contact`,
`a = axle_to_hinge`일 때 사용 식은 다음과 같다.

```text
q(k) = atan(k d) + asin(k a / sqrt(1 + (k d)^2))
```

기본 계획은 측정된 hinge 범위의 80%만 사용한다. 현재 길이에서 이에 해당하는 곡률 한계는
약 `0.6795 m⁻¹`, 최소 반경은 약 `1.47 m`다. 반대편 로봇에는 대칭 부호의 hinge 각을 적용한다.
양쪽 axle pose를 생성한 뒤 곡률·힌지 각·차동구동의 횡방향 변위 비율을 확인한다.

힌지 각은 저속 준정적 평형 모델이다. 이 값만으로 수동 베어링이 실제 운반 중 같은 각을
따른다고 보장할 수 없다.

## 상자 pose에서 경로 생성

`cooperative_object_path_planner_node`는 `geometry_msgs/PoseStamped` 시작·목표 pose를 입력받는다.
시작·목표 yaw를 접선으로 갖는 cubic Bezier 후보와 endpoint pose에 맞는 원호 후보를 만들고,
각 후보를 양 로봇 axle 경로로 변환해 기구학 제약을 확인한다. 통과한 후보 중 곡률 에너지와
길이 비용이 낮은 경로를 고른다.

작업 경계와 장애물은 설정으로 전달한다.

- `obstacle_circles`: `[x, y, radius, ...]` 형식의 정적 원형 장애물 목록
- `workspace_bounds`: `[xmin, xmax, ymin, ymax]` 형식의 사각 작업영역
- 로봇은 axle 중심 반경 0.32 m의 원, 상자는 중심 반경 0.10 m의 원으로 검사한다.
- 장애물·경계에는 0.05 m 여유를 더한다.

이 원형 외곽 검사는 실제 차체 회전 외곽을 간단히 근사한다. 현재 후보 검색기는 일반적인
전역 최적화/격자 경로 계획기가 아니며, 임의의 장애물 배치에 경로가 있다고 보장하지 않는다.
센서 map, Nav2 costmap, 동적 장애물과 충돌 지도를 연결하지 않았다.

## Leader `/plan`에서 경로 변환

실제 Nav2의 `/plan`을 입력으로 쓰려면 다음 launch를 실행한다.

```bash
ros2 launch cooperative_mission cooperative_path_preview_from_leader.launch.py
```

`cooperative_leader_path_adapter_node`는 `/plan`을 Leader axle 경로로 읽는다. 경로 방향과 pose의
곡률로 힌지 각을 초기화하고, 상자 중심 경로 곡률을 반복 추정한다. 상자 경로에서 Leader axle
경로를 다시 생성해 원 입력과 위치 2 cm, yaw 2° 이내로 맞는지 확인한다. curvature, hinge,
횡방향 변위 또는 round-trip 검사가 실패하면 해당 경로를 거부한다.

`/plan`의 pose orientation은 경로 진행 방향을 따라야 한다. Leader axle 경로를 상자 중심 경로로
그대로 취급하거나 `/plan`을 object path 토픽에 직접 연결하지 않는다.

## Follower 경로 방향과 pure-pursuit 제어

`cooperative_path_preview_node`는 상자 중심 경로에서 두 axle 경로를 만든다. 각 경로 segment의
이동 벡터를 로봇 heading에 투영해 `FORWARD` 또는 `REVERSE`를 판정한다. 서로 마주 보는 두 로봇이
같은 방향으로 상자를 옮기는 이 시연에서는 Leader가 전진할 때 Follower가 후진한다.

Follower mission executor는 Follower 경로와 `/follower/odom/raw`를 입력받는다.
경로와 odometry frame이 다르면 TF를 조회해 pose를 path frame으로 바꾼 뒤 pure pursuit를 계산한다.
진행 방향에 따라 `linear.x` 부호를 적용하며, 경로 곡률이 커지면 설정된 횡가속도 한계로 속도를
낮춘다. odometry timeout, 경로 오차 초과, 잘못된/mixed path 방향이면 추종을 중단하고 0 속도를
낸다. 최대 속도 기본값 0.08 m/s는 단독 실차 시험에서 동작을 확인한 값이다.

제어 명령은 기존 selector의 COOPERATION 입력에 발행한다. 상태는 다음 토픽에서 확인한다.

```text
/follower/mission/cmd_vel             geometry_msgs/Twist
/follower/path_tracking/status        std_msgs/String
```

mission executor가 유일한 COOPERATION 입력 발행기다. 추종 시작 시 velocity guard를 켠 다음
selector를 `COOPERATION`으로 전환한다. 정지 시 0 속도를 보내고 selector를 `STOP`으로 바꾼 뒤
guard를 해제한다. 완료·오차 초과·stale odometry·TF 오류 때도 이 정지 절차를 수행한다.

```bash
ros2 service call /follower/path_tracking/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /follower/path_tracking/status
ros2 service call /follower/path_tracking/enable std_srvs/srv/SetBool "{data: false}"
```

## ROS 토픽

| 노드 | 입력 | 출력 |
|---|---|---|
| `cooperative_object_path_planner_node` | `/cooperation/object_start`, `/cooperation/object_goal` | `/cooperation/object_path`, `/cooperation/object_path_planner/status` |
| `cooperative_leader_path_adapter_node` | `/plan` | `/cooperation/object_path`, `/cooperation/leader_path_adapter/status` |
| `cooperative_path_preview_node` | `/cooperation/object_path` | `/cooperation/leader_path_preview`, `/cooperation/follower_path_preview`, `/cooperation/path_preview/status` |
| `follower_mission_node` (path tracking mode) | `/cooperation/follower_path_preview`, `/follower/odom/raw`, TF | `/follower/mission/cmd_vel`, `/follower/path_tracking/status` |

planner demo는 설정된 workspace·장애물 MarkerArray를 `/visualization_marker_array`로 발행한다.
RViz에서 Leader axle은 주황색, Follower axle은 청록색, 작업 경계는 회색, 장애물은 빨간색이다.

## 실행

### 시작·목표 pose와 3×4 m 시연 환경

```bash
ros2 launch cooperative_mission cooperative_path_preview_demo.launch.py
rviz2 -d "$(ros2 pkg prefix cooperative_mission)/share/cooperative_mission/rviz/cooperative_path_preview.rviz"
```

demo 입력은 상자 시작 `(-0.8, -1.5, 0 rad)`, 목표 `(0.7, 0, π/2)`이며, 작업영역은
`x=[-1.5, 1.5]`, `y=[-2, 2]`다. 정적 장애물 중심은 `(-0.8, 0)`, 반경 0.18 m다.
이 90° 원호 시연은 길이 약 2.36 m, 반경 약 1.5 m, 최대 곡률 약 `0.667 m⁻¹`, 최대 hinge
각 약 11.8°로 계산된다.

### 실제 Leader `/plan`에서 Follower 경로 만들기

```bash
ros2 launch cooperative_mission cooperative_path_preview_from_leader.launch.py
```

Follower 기본 launch도 경로 adapter와 axle 경로 계산기를 자동 실행한다. 위 별도 preview launch는
진단용이다. 실차 단독 추종은 Follower 기본 launch에서 경로와 odometry가 준비된 것을 확인한 뒤
서비스로 시작한다. Leader 경로를 협동 운반 중 자동 승인·실행하는 임무 통합은 별도 작업이다.

## 확인 결과 및 미완료 작업

- `colcon build --packages-select cooperative_mission`을 ROS Humble에서 완료했다.
- 이전 시뮬레이션에서는 synthetic 90° Leader path를 `/plan`으로 넣어 adapter의 상자 경로 역산,
  round-trip 확인, 양쪽 axle 경로 발행을 확인했다. 결과는 `leader_drive=FORWARD`,
  `follower_drive=REVERSE`였다. 실차 Leader Nav2의 `/plan` 입력은 아직 검증하지 않았다.
- 2026-10-04 실차에서 Follower mission executor의 추종 출력을 기존 selector, velocity guard,
  STM32 bridge에 연결했다. Follower 현재 odometry에서 시작하는 합성 `odom` 프레임 후진 경로를
  실행했고, 목표 25 cm에 대해 실제 이동은 `dx≈-0.001 m`, `dy≈-0.216 m`, heading 변화 약 0.3°였다.
  목표 허용오차 5 cm에 도달해 자동 정지했다. guard 비활성화와 `/follower/safe_cmd_vel=0`도 확인했다.
- 같은 날 가상 Leader `/plan`을 격리 토픽에 넣고 adapter → object path → 양쪽 axle path →
  Follower pure-pursuit 계산을 거친 뒤, 생성된 Follower 경로를 실차에서 추종했다. 직선 입력의
  curvature 0, hinge 0°, Leader `FORWARD`, Follower `REVERSE`를 확인했다. 실차 이동은
  `dx≈+0.002 m`, `dy≈-0.213 m`, yaw 변화 약 0.08°였고, preview 속도는 `-0.08 m/s`였다.
  목표 허용오차에 도달해 자동 정지하고 guard off 및 safe command 0을 확인했다.
- 이 시험은 가상 plan으로 경로 변환과 Follower 실차 추종을 확인했다. 실제 Leader Nav2 publisher의
  `/plan`과 두 Orin 간 DDS/frame/TF 연결은 아직 확인하지 않았다.
- 별도로 Follower 단독 후진 곡선 경로(길이 0.35 m, 계획 반경 0.8 m)를 실차 추종했다. odometry
  chord 약 0.316 m, yaw 변화 `+17.47°` 후 goal tolerance에서 정지했다. 물체를 두 로봇이 잡은
  상태의 곡선 운반은 아니다.
- 실제 Leader Nav2 경로·두 Orin TF/DDS 연결, 물체 결합 곡선 운반, 경로 오차 정량 검증,
  비상정지, 장애물 회피, Leader 승인과 협동 모드 연동은 미검증이다. 테스트 suite는 이번
  실차 작업에서 실행하지 않았다.
- 상세 현황, 선행 조건, 남은 작업과 시험 순서는
  [협동 경로 추종 개발 진행상황](COOPERATIVE_PATH_TRACKING_PROGRESS.md)에 기록했다.

기존 cooperative mission의 Leader 속도 추종은 그대로 남아 있다. 새 경로 추종은 Follower `IDLE`
상태에서 서비스를 호출하는 독립 단독 시험 모드다. Leader mission 상태나 물체 파지/리프트 승인을
검사하지 않으므로 협동운반에 자동 투입하면 안 된다.
