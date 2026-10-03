# 협동운반 경로 변환 알고리즘 초안

## 진행 상황 (2026-10-03)

- 완료: 강체 그립 관계를 이용한 리더 pose → 팔로워 pose 변환 및 회전항을 포함한
  속도/차동구동 가능성 계산 로직을 추가했다.
- 완료: `/plan` 기반 팔로워 경로 미리보기 ROS 노드, 실행 launch, 기본 비활성 설정을
  추가했다. 미리보기 노드는 구동 명령을 보내지 않는다.
- 미완료: 로봇·상자 실측값 입력, ROS 빌드 및 런타임 연결 확인, RViz 시각화 확인.
- 다음: 실측값 보정 → 미리보기 검토 → 상자/로봇 footprint 충돌검사 → 별도 구동 연동과
  안전 검토 순으로 진행한다. 지금 단계에서는 실차 운반 경로를 실행하지 않는다.

## 목적

리더의 계획 경로를 물체의 경로로 해석하고, 양쪽 그립 관계를 이용해 팔로워의 목표
경로와 속도를 계산한다. 팔로워가 단순히 리더 `linear.x` 부호만 반전하면 회전운동에
따른 두 로봇 위치 차이를 반영할 수 없으므로, 고정된 강체 그립 가정을 기준으로 한다.

## 기하 모델

각 변환 `T_AB`는 B 좌표를 A 좌표로 옮긴다. 로봇 CAD와 상자 크기에서 다음 변환을
측정·계산한다.

| 변환 | 의미 |
|---|---|
| `T_L_GL` | 리더 base에서 리더 그리퍼 접촉 프레임 |
| `T_O_GL` | 상자 중심 프레임에서 리더 그리퍼 접촉 프레임 |
| `T_O_GF` | 상자 중심 프레임에서 팔로워 그리퍼 접촉 프레임 |
| `T_F_GF` | 팔로워 base에서 팔로워 그리퍼 접촉 프레임 |

리더의 계획 pose가 `T_WL`이면:

```text
T_WO = T_WL · T_L_GL · inverse(T_O_GL)
T_WF = T_WO · T_O_GF · inverse(T_F_GF)
```

따라서 경로의 모든 리더 pose를 같은 식으로 변환하면 팔로워 기준 경로가 된다.
두 그리퍼가 상자에 대해 미끄러지지 않는다는 전제에서 경로는 리더와 팔로워의
고정된 상대 pose를 유지한다.

`GraspGeometry`와 `leader_pose_to_follower` / `transform_leader_path`가 이 2D 변환을
구현한다. 기하 엔진은
[`formation.py`](../src/cooperative_mission/cooperative_mission/formation.py)에 있다.

## ROS 경로 미리보기

`cooperative_path_preview_node`는 `/plan`을 구독해 위 기하 변환을 적용하고
`/cooperation/follower_path_preview` (`nav_msgs/Path`)로 내보낸다. 입력 계획과 출력은
`leader_odom_frame`(기본 `odom`)이어야 한다. `/visual_slam/tracking/odometry`와
`/nav2/cmd_vel`은 현재 회전 기준 속도 가능성 진단에만 쓰며, 변환 경로 자체는 `/plan`에서
계산한다. 결과 상태는 `/cooperation/path_preview/status`, 현재 속도 미리보기는
`/cooperation/follower_twist_preview`에서 확인한다.

```bash
ros2 launch cooperative_mission cooperative_path_preview.launch.py
```

기본 설정은 `geometry_ready: false`이고 모든 치수가 0이므로 의도적으로 경로를 출력하지
않는다. 실제 기구를 측정해 `config/cooperative_path_preview.yaml`의 상자 크기와 네 pose를
채운 뒤에만 `geometry_ready: true`로 바꾼다. 이 노드는 시각화·진단 전용이며 구동 명령을
발행하지 않는다. 또한 아직 변환 경로의 충돌 검사, 팔로워 Nav2 goal 추종, 경로 재계획 연동은
구현되지 않았다.

## 속도 변환과 주행 가능성

리더의 body twist를 `(v_L, 0, omega)`라 하고 리더 base에서 팔로워 base까지의
상대 벡터를 리더 좌표로 `(dx, dy)`라 하면 팔로워 위치의 세계 좌표 속도는
리더 원점 속도에 회전 항 `omega × (dx, dy)`를 더해 얻는다. 이를 팔로워 base 좌표로
회전해 `(v_Fx, v_Fy, omega_F)`를 구한다.

차동구동 팔로워는 옆 방향 속도 `v_Fy`를 만들 수 없다. 따라서 `abs(v_Fy)`가 허용치보다
크면 해당 리더 회전 명령은 현재 고정 그립 구조에서 실행 불가로 판단해야 한다. 이때
명령을 임의로 잘라 보내면 상자에 비틀림이 걸릴 수 있으므로 정지 후 더 완만한 경로를
요청한다. `leader_twist_to_follower`는 종방향 명령과 함께 필요한 횡방향 속도를 돌려주며,
호출자가 주행 가능성을 판정하도록 한다.

## 경로 입력과 계획기 제약

- 리더의 미래 경로 입력은 `nav_msgs/Path`의 pose sequence다. `/nav2/cmd_vel`은 현재 시점의
  controller 명령일 뿐, 미래 경로의 대체물이 아니다.
- 팔로워에서 쓸 경로는 `T_WF`를 follower가 사용할 좌표계로 변환해야 한다. 양 로봇의
  `map`/`odom`이 같은 좌표계라는 보장이 없으면, 파지 완료 시점의 상대 pose를 기준으로
  초기 정렬을 만들고 상대 경로를 전달한다.
- Nav2가 리더 footprint만 고려해 만든 경로를 그대로 실행하면 상자나 팔로워가 장애물에
  닿을 수 있다. 다음 단계에서는 상자와 두 차체의 footprint 합집합을 costmap에 반영하거나,
  변환 경로를 footprint 전체로 검사하고 충돌 시 리더 경로 재계획을 요청해야 한다.
- 물체가 양쪽 그리퍼에서 회전할 수 있거나 접촉점이 미끄러지면 강체 모델은 맞지 않는다.
  실제 그리퍼 관절 유격과 물체 고정 방식을 확인한 후 rigid-grasp 가정을 유지할지 정한다.

## 현재 저장소에서 확인된 연결 상태

Nav2는 `/plan` 경로 생성이 확인됐지만 `/nav2/cmd_vel`은 wheel bridge와 분리돼 있고,
실차 주행은 아직 검증되지 않았다. 기존 미션 수행 경로는 `/plan`을 추종하지 않고,
리더가 생성한 제한 속도 명령으로 기본 직진 동작을 수행한다. 새 경로 미리보기 노드는 기존
미션을 바꾸지 않는 독립 진단 단계다.

실제 운반 경로를 켜기 전 필요한 측정값은 상자 길이·폭·높이, 양쪽 접촉점의 상자 중심
기준 pose, 각 로봇 base 기준 그리퍼 pose, 두 차체 footprint, 그리고 좌표계 간 시작 시
정렬이다. 높이는 2D 경로 기하에는 들어가지 않지만 수직 간섭·리프트 여유 검사에 필요하다.
