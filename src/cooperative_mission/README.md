# cooperative_mission

리더·팔로워 두 로봇이 **하나의 물체를 양쪽에서 잡고, 동시에 들어 올린 뒤, 같은 방향으로
1초간 직진**하는 전체 시나리오를 조정하는 ROS 2(Humble) 패키지다. 기존 인식·접근·안전
파이프라인은 그대로 재사용하고, 그 위에 두 개의 미션 노드만 추가한다.

| 단계 | 리더 상태 | 팔로워 상태 | 사용하는 기존 구성요소 |
|---|---|---|---|
| 1. 물체 태그(QR) 인식 | `LEADER_SEARCH` → `LEADER_APPROACH` | `IDLE` | `rescue_robot_apriltag`, `leader_approach_control` |
| 2. 리더 파지 | `LEADER_GRASP` | `IDLE` | `rescue_robot_tools/dynamixel_orin_node` |
| 3. 팔로워 반대편 이동·파지 | `FOLLOWER_APPROACH` | `REPOSITION`(선택) → `SEARCH` → `APPROACH` → `GRASP` → `GRASPED` | `follower_supply_perception`, `follower_approach_control`, `follower_command_selector`, `follower_control` |
| 4. 동시 리프트 | `LIFT_PREPARE` → `LIFT` | `LIFT_READY` → `LIFTING` → `LIFTED` | 양쪽 RX-64 |
| 5. 1초 협동 직진 | `TRANSPORT_PREPARE` → `TRANSPORT` → `TRANSPORT_FINISH` → `DONE` | `TRANSPORT_READY` → `HOLD` | 양쪽 velocity guard, STM32 bridge |

"QR 코드"는 저장소 전체가 사용하는 **AprilTag tag36h11**(5 cm, ID 0)로 구현한다. 물체의
리더 쪽 면과 팔로워 쪽(반대) 면에 각각 태그를 붙인다. 두 면 모두 ID 0이면 기본 설정 그대로
동작한다.

STM32 펌웨어는 **변경하지 않았다.** 바퀴 속도·watchdog는 기존 `CMD_VELOCITY` 경로를,
그리퍼(RX-28 집게 / RX-64 리프트)는 Jetson U2D2 경로를 그대로 사용한다.

## 구조

```text
Leader Orin                                             Follower Orin
───────────                                             ─────────────
apriltag_approach ─┐                                    apriltag_approach ─┐
approach_controller┤ /leader/approach/cmd_vel_raw        approach_controller┼→ /follower/approach/cmd_vel_raw ─┐
                   ▼                                                        │                                 │
        mission_coordinator ──/leader/mission/cmd_vel_raw→ velocity_guard   │ mission_executor                │
          │  │  │                    → mission/cmd_vel_safe → selector      │  └→ /follower/mission/cmd_vel ─┐│
          │  │  │                                      → /leader/cmd_vel → STM32                             ││
          │  │  └─ /leader/dynamixel/command → Dynamixel                   │                                ▼▼
          │  │                                                             │        command_selector (STOP/APPROACH/COOPERATION)
          │  └─ /mission/follower_command ─────── DDS ──────────────────────►│                  ▼
          │  ◄─ /follower/mission/status ─────── DDS ───────────────────────┤        velocity_guard → /follower/safe_cmd_vel → STM32
          └─ /cooperation/target_velocity, /mission/state ─ DDS ────────────►│  /follower/dynamixel/command → Dynamixel
```

* 리더 `mission_coordinator`가 리더 velocity guard 입력의 **단일 소유자**다. approach
  controller 출력은 `LEADER_APPROACH` 상태에서만(후진 성분 제외) 전달한다.
  guard의 safe 출력은 리더 selector의 `MISSION` 입력을 거치며, selector만 최종
  `/leader/cmd_vel`을 발행한다.
* 팔로워 `mission_executor`는 selector의 `COOPERATION` 입력(`/follower/mission/cmd_vel`로
  remap)을 소유하고, selector `source_mode`·approach enable·guard enable을 서비스로 전환한다.
* 모든 리더→팔로워 명령은 `(session, seq)`가 붙은 JSON(`std_msgs/String`)이며 ACK가 올 때까지
  0.2 s마다 재전송된다. 팔로워는 같은 seq를 한 번만 실행한다.

## 동기화 방법

* **동시 리프트**: 리더가 `LIFT`를 보내면 팔로워는 수신 즉시 RX-64를 올리고 ACK를 보낸다.
  리더는 그 ACK를 받는 순간 자신의 RX-64를 올린다. 따라서 두 팔의 시작 차이는 DDS 한 홉
  (수 ms)이며, 팔로워가 준비되지 않으면 리더는 절대 혼자 들어 올리지 않는다.
* **1초 협동 직진**: 리더가 사다리꼴 공통 속도(기본 0.05 m/s, 0.20 m/s², 램프 포함 1.0 s)를
  리더 좌표로 `/cooperation/target_velocity`에 50 Hz로 스트리밍한다. 서로 마주 보고 잡고
  있으므로 팔로워는 `linear.x` 부호를 뒤집어(`leader_facing_opposite: true`) 추종한다.
  프로파일 가속도가 두 guard의 가속 제한(0.25 m/s²)보다 작아 guard가 프로파일을 변형하지
  않는다. 리더는 팔로워 중계 지연을 보상하기 위해 자기 명령을 `transport_leader_delay`(20 ms)
  만큼 늦춘다.

## 안전 정책

* `/mission/start` 전에는 두 로봇 모두 approach/guard가 꺼져 있다. 리더 selector는
  `MISSION`에서 zero를 유지하고 팔로워 selector는 `STOP`이다.
* 시작 조건: 리더 서비스·그리퍼 노드 준비, `/cooperation/target_velocity`·
  `/leader/mission/cmd_vel_raw`의 다른 publisher 없음, 팔로워 상태 신선·`IDLE`·ready.
* 서비스 응답 실패/무응답(2 s), 그리퍼 `ERROR`, 태그 재탐색 초과, 각 단계 timeout,
  팔로워 상태 끊김(운반 중 0.35 s, 그 외 1 s), 리더 heartbeat 끊김(팔로워가 움직이는 중 1 s)
  → `FAULT`: 속도 0, approach/guard off, 팔로워 selector `STOP`. **그리퍼는 계속 잡고 있다.**
* 기존 안전 경계(velocity guard 제한·watchdog, selector freshness, STM32 200 ms watchdog)는
  그대로 유지된다.

## 서비스

| 서비스 (`std_srvs/Trigger`) | 동작 |
|---|---|
| `/mission/start` | IDLE에서 전체 시나리오 시작 |
| `/mission/abort` | 즉시 FAULT(두 로봇 정지, 그리퍼 유지) |
| `/mission/release` | DONE/FAULT에서 두 로봇이 함께 내려놓고(RX-64 `lower_raw`) 집게를 연 뒤 IDLE |
| `/mission/reset` | IDLE/DONE/FAULT → IDLE, 팔로워에도 RESET (그리퍼는 움직이지 않음) |

## 주요 파라미터

리더 `config/leader_mission.yaml`, 팔로워 `config/follower_mission.yaml`에 설명과 함께 있다.
launch 인자(`transport_direction`, `transport_speed`, `transport_duration`, 그리퍼 raw 값)는
YAML보다 우선한다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `transport_direction` | `forward` | 리더 base_link 기준 운반 방향 (`backward`면 launch가 리더 guard `allow_reverse`를 켬) |
| `transport_speed` | 0.05 m/s | 순항 속도 (≤ 0.10) |
| `transport_duration` | 1.0 s | 램프 포함 전체 이동 시간 |
| `lift_raw` / `lower_raw` | Leader 300 / 600, Follower 20 / 270 | RX-64 들어올림 / 내려놓기 위치; Follower raw 20=최고, 270=최저 |
| `lift_duration` | 3.5 s | RX-64 이동 대기 (속도 50에서 600→300 약 2.6 s) |
| `reposition_segments` (팔로워) | `[""]` | 반대편까지 돌아가는 odometry 기동, 예 `["turn:90","drive:0.6","turn:-90"]` |

리더 guard는 미션 전용 설정(`config/leader_velocity_guard_mission.yaml`)을 쓴다. 기존 대비
`command_topic`, `max_linear_speed 0.05→0.10`, `max_linear_acceleration 0.10→0.25`만
다르며 approach controller 자체 속도 제한(0.05 m/s)은 그대로다.

## 시험

```bash
colcon test --packages-select cooperative_mission && colcon test-result --verbose
# 또는 ROS 없이
cd src/cooperative_mission && python3 -m pytest -q test
```

`test/sim_world.py`는 두 상태기계를 지연이 있는 가상 DDS 링크와 1차원 로봇 모델로 묶어
정상 시나리오, 동시 리프트(시작 차 ≤ 2 홉), 1초·동일 방향 운반, 링크 단절, 서비스 실패,
재전송 멱등성, release/reset 흐름을 검증한다.

실행 절차는 [협동 미션 실행 가이드](../../docs/COOPERATIVE_MISSION_RUN_GUIDE.md)를 따른다.
협동운반 경로 생성 알고리즘은
[협동운반 경로 변환 알고리즘](../../docs/COOPERATIVE_TRANSPORT_PATH_ALGORITHM.md)을 참고한다.
ROS 진단 노드는 `ros2 launch cooperative_mission cooperative_path_preview.launch.py`로
실행할 수 있다. 이 노드는 `/cooperation/object_path`의 상자 중심 경로를 받아, passive yaw
힌지 각도 제한과 차동구동 기구학을 적용한 두 로봇의 axle 경로를 생성한다. 현재 경로 출력은
미리보기용이며 실차 구동에는 연결되지 않는다.
경로 확인용 RViz 설정은 `rviz/cooperative_path_preview.rviz`이며, 실행 노드와 별도로
`rviz2 -d "$(ros2 pkg prefix cooperative_mission)/share/cooperative_mission/rviz/cooperative_path_preview.rviz"`
로 연다.
실제 Nav2 없이 3×4 m 시연 경로를 생성하려면
`ros2 launch cooperative_mission cooperative_path_preview_demo.launch.py`를 사용한다. 이 launch는
합성 상자 시작·목표 pose를 발행하고, 회전 안쪽 원형 장애물과 작업 경계를 고려해 힌지·차동구동 조건을
통과한 상자 경로를 `/cooperation/object_path`로 발행한다. RViz에 작업 경계와 장애물도 표시한다.
장애물은 설정 파일의 정적 원형 모델이며 Nav2 `/plan`이나 실제 costmap은 사용하지 않는다.
리더 Nav2 `/plan`에서 팔로워 경로를 미리 보려면
`ros2 launch cooperative_mission cooperative_path_preview_from_leader.launch.py`를 사용한다.
이 노드는 리더 axle 경로에서 상자 중심 경로를 역산한 뒤, 양쪽 로봇 경로와 전진·후진 방향을
계산한다. Follower odometry가 들어오면 pure-pursuit 추종 속도도 계산하지만
`/cooperation/follower/path_tracking/cmd_vel_preview`에만 발행하며 실차 명령과는 분리돼 있다.
