# Leader 1차 시나리오 개발 로드맵

## 목적

현재 개별 검증된 Leader 기능을 보존하면서 Mapping에서 AprilTag 파지와 생존자 전달까지
이어지는 첫 단독 시연을 단계별로 통합한다. 각 phase는 hardware-free 검증을 먼저 하고,
실차 검증 결과와 코드 검증 결과를 따로 기록한다.

## 현재 검증된 기능

- VSLAM: 공식 Mapping script에서 D435 infra stereo를 사용하며 `map → odom → base_link` TF를 제공한다.
- nvblox: D435 depth/color로 3D mesh 및 ESDF를 생성하고 Nav2 costmap 입력을 제공한다.
- Survivor: YOLO/depth, camera/map 위치, RViz marker와 mission-runtime persistent Registry가 있다. 공유 D435에서 VSLAM/nvblox와 함께 실행한 수동 확인 기록이 있다.
- AprilTag: detection, tag TF, base alignment와 raw approach controller가 있다. 기존 velocity guard는 timeout, clamp, slew limit, reverse 차단과 enable gate를 제공한다.
- Gripper/lift: `ALIGNED → RX-28 CLOSE → RX-64 LIFT → DONE` sequence가 있다. 개별 actuator 확인과 전체 grasp/transport 성공은 구분한다.
- Motion arbitration: Leader selector가 `STOP/TELEOP/APPROACH/NAV2`를 제공한다. 자동 검증은 dummy Twist이며 real Nav2 drive는 검증되지 않았다.
- Follower selector/cooperation은 별도 기능이며 이 Leader 단독 시연에는 포함하지 않는다.

## 최종 Leader 1차 시연 시나리오

```text
START
 → D435 + VSLAM + nvblox + Survivor
 → TELEOP Mapping
 → Survivor 발견 및 map 좌표/Registry ID 저장
 → 키보드로 AprilTag 물품 앞까지 이동
 → 명시적 Mapping 완료 / Mission 시작 trigger
 → TELEOP 중단, selector APPROACH 인계
 → AprilTag 정밀 정렬: ALIGNED
 → RX-28 CLOSE → RX-64 LIFT → DONE
 → 선택된 Survivor map pose를 delivery goal로 변환
 → selector NAV2, Survivor 근처까지 이동
 → RX-64 LOWER → RX-28 OPEN
 → MISSION DONE
```

Mapping 중 VSLAM/map session은 유지한다. 위 전체 시퀀스는 목표이며 완료 단계는 아래 phase별
검증 결과로 판정한다.

## 이번 버전에서 의도적으로 생략하는 기능

- Supply Registry, 물품 map waypoint, AprilTag 물품 위치 저장
- Nav2 물품 자동 접근
- Frontier exploration
- Follower 협동 운반 및 Heavy/Light 자동 분기

첫 시연에서는 사용자가 keyboard teleop으로 물품 앞까지 이동한다. 물품 waypoint 자동화는
별도 후속 기능으로 다룬다.

## 전체 architecture

### 공유 D435

```text
run_vslam_mapping.sh owns one D435
 ├─ VSLAM: infra1/infra2
 ├─ nvblox: depth/color
 ├─ Survivor: RGB/depth shared topics
 └─ AprilTag: RGB shared topic, Survivor-owned rectification
```

Mapping script가 D435와 `/leader/stm32_bridge`를 소유한다. AprilTag 통합 launch는
`start_camera:=false`, `start_camera_processing:=false`,
`start_robot_state_publisher:=false`, `use_stm32_bridge:=false`로 기존 owner를 재사용한다.

### Motion command

```text
/leader/teleop/cmd_vel ───────┐
/leader/approach/cmd_vel_safe ├→ leader_command_selector → /leader/cmd_vel → STM32
/nav2/cmd_vel ────────────────┘

/leader/approach/cmd_vel_raw → velocity_guard → /leader/approach/cmd_vel_safe
```

Selector 기본 mode는 `STOP`; Mapping script는 `TELEOP`을 명시적으로 시작한다. AprilTag guard의
기존 안전 처리는 selector 앞에 유지된다. mode는 `/leader/command_selector`의 `source_mode`
parameter로 바꾸고 `/leader/command_selector/status`에서 `STOP`, `WAITING_*`, `ACTIVE_*`,
`STALE_*`를 확인한다.

### Survivor, AprilTag, Nav2와 Mission

Survivor Registry는 map frame persistent track을 제공한다. 향후 delivery adapter가 선택된
track pose를 안전 offset을 포함한 Nav2 goal로 변환한다. AprilTag approach와 Nav2는 selector의
상호 배타 source다. Mission Coordinator는 이후 component start/stop, handoff, gripper trigger,
fault 복구를 조정한다. 이번 단계에는 coordinator가 없다.

## 단계별 개발 계획

| Phase | 목표·입력·출력 및 예상 component | 완료 조건 | 위험요소 |
|---|---|---|---|
| 0. Baseline Freeze | clean HEAD와 annotated tag 기록 | tag가 정확한 commit을 가리키며 사용자 변경을 보존 | dirty 변경을 baseline으로 오인 |
| 1. Shared resources | Mapping D435/TF/STM32를 Survivor와 AprilTag가 공유; bringup launch와 run script | 단일 owner, camera/topic/TF graph에 치명적 중복 없음 | camera processing과 TF 중복 |
| 2. Command Selector | TELEOP/APPROACH/NAV2 input → 단일 `/leader/cmd_vel`; selector, teleop remap, guard remap | source isolation, switch zero, stale timeout, shutdown zero 통과 | 이전 command 잔류 또는 selector 우회 |
| 3. Nav2 motor 연결 | `/nav2/cmd_vel`에서 실제 STM32 주행 | E-stop/bridge watchdog 아래 0.5 m, 1 m, 회전, 장애물 stop 시험 통과 | 과주행, localization/controller fault |
| 4. Survivor pose → delivery goal | confirmed track/map pose → safe Nav2 goal adapter | frame/offset 정확, invalid/lost track 거부 | map/odom 혼동, 사람 근접 goal |
| 5. Mapping → grasp | Mapping 완료 trigger → APPROACH → ALIGNED → CLOSE/LIFT | Mapping 유지 중 teleop 정지, 기존 sequence가 순서대로 1회 실행 | 두 source 동시 활성, 중복 파지 trigger |
| 6. Gripper DONE → delivery | DONE + valid goal → NAV2 | fresh goal로 주행, cancel/failure는 STOP/FAULT | 적재 후 주행 불안정, stale goal |
| 7. LOWER/OPEN | 도착 확인 후 RX-64 LOWER → RX-28 OPEN | 안전 도착 gate 후 순서 및 완료 확인 | 조기 release, 중복 command |
| 8. Minimal Mission Coordinator | MAPPING_TELEOP, READY_AT_SUPPLY, APRILTAG_APPROACH, GRASPING, LIFTING, NAV_TO_SURVIVOR, DELIVERING, MISSION_DONE, FAULT | 허용 transition만, fault 시 STOP, 재실행 안전 | 상태와 hardware 불일치 |
| 9. End-to-End 반복 | bringup/logging/checklist | 합의된 여러 회차 전 과정과 중단/복구 시험 pass | 타이밍/누적 상태 오류 |

### Phase 0 — Baseline Freeze

입력은 현재 HEAD와 clean/dirty 상태, 출력은 immutable annotated tag와 commit 기록이다.
phase 시작과 종료 때 status 및 tag target을 확인한다.

### Phase 1 — Shared D435/STM32 integration

Mapping launcher가 D435와 STM32 bridge owner다. Survivor는 shared camera image/depth를 쓰고
RGB CameraInfo bridge/rectifier를 제공한다. AprilTag 단독 launch 기본값은 기존처럼 자체
camera/processing/TF/bridge를 실행하고, Mapping 공유 launch에서는 ownership arguments로 이를
끄고 기존 topic을 사용한다. VSLAM, nvblox, Survivor, AprilTag topic과 TF owner가 함께 살아야 한다.

### Phase 2 — Leader Command Selector

Inputs는 `/leader/teleop/cmd_vel`, guard 뒤 `/leader/approach/cmd_vel_safe`, `/nav2/cmd_vel`;
output은 `/leader/cmd_vel`이다. Generic STOP, runtime `source_mode`, source별 stale timeout,
mode 전환 zero와 shutdown zero가 완료 조건이다. Mapping script는 TELEOP을 명시하고 final
topic에는 selector 외 publisher가 없어야 한다.

### Phase 3 — Nav2 `/nav2/cmd_vel` 실제 STM32 주행 검증

Dummy selector test 뒤 실 wheel write를 별도 안전 절차에서 시작한다. E-stop과 bridge watchdog을
확인하고 wheels-lifted 및 낮은 속도부터 0.5 m, 1 m, 제자리 회전, 장애물 앞 stop을 반복 측정한다.
localization과 valid controller trajectory도 확인한다.

### Phase 4 — Survivor map pose → safe delivery goal

선택된 confirmed Registry track, map pose, map/odom TF, footprint와 safe offset을 사용해 goal을
만든다. frame 변환, lost/invalid track, 불가능한 goal, Nav2 rejection을 시험하며 사람 위치 자체를
무조건 endpoint로 쓰지 않는다.

### Phase 5 — Mapping 종료 trigger → APPROACH → CLOSE/LIFT

명시 trigger가 teleop을 중단시키고 selector를 APPROACH로 handoff한다. VSLAM/map session은
유지한다. 기존 AprilTag algorithm과 `ALIGNED → CLOSE → LIFT → DONE` sequence를 재사용한다.

### Phase 6 — Gripper DONE → Survivor delivery

기존 sequence DONE 및 유효 goal을 확인한 뒤 selector를 NAV2로 바꾸고 NavigateToPose를 시작한다.
cancel, stale goal, lost track, Nav2 failure는 STOP/FAULT로 간다.

### Phase 7 — LOWER/OPEN

정지 및 도착 gate 후 RX-64 LOWER, RX-28 OPEN 순으로 실행하고 완료 상태를 확인한다. hardware
검증 전에 actuator raw goal이나 sequence timing을 임의 변경하지 않는다.

### Phase 8 — Minimal Mission Coordinator

기존 component의 검증된 trigger/state API만 조정한다. 불가능 transition과 failure에는 STOP 및
명시 FAULT를 정의하고 sensor/registry 알고리즘을 중복 구현하지 않는다.

### Phase 9 — End-to-End 반복 검증

mode, Registry ID/map pose, tag/alignment, gripper, goal, Nav2 result와 fault/recovery를 하나의
기록으로 연결한다. 완료는 한 번의 시연 성공이 아니라 합의된 반복 pass와 안전 중단 확인으로
판정한다.

## 이후 확장

- Supply Registry 및 AprilTag 물품 map pose 저장은 operator가 물품 앞까지 이동하는 v1 뒤에 추가한다.
- 물품 waypoint와 Nav2 자동 접근은 Supply Registry를 검증한 뒤 별도 단계로 진행한다.
- Frontier exploration은 수동 Mapping workflow가 안정화된 뒤 검증한다.
- Follower 협동 운반은 Leader 단독 시나리오와 별도 release gate를 둔다.

## 현재 상태와 알려진 위험

- Phase 0/1 resource wiring은 launch tests 및 motor-free launch graph로 확인했다. Jetson 동시
  D435 session regression은 남아 있다.
- Phase 2 selector는 unit/dummy topic으로 확인했다. 실제 Nav2 wheel drive는 검증하지 않았다.
- Mapping 중 AprilTag launch는 이미 실행한 selector에 `start_command_selector:=false`를 줘야 한다.
- `/leader/cmd_vel`에 selector 외 direct publisher가 생기면 single-owner 보장이 깨진다.
- Survivor delivery offset, full gripper transport, release, Mission Coordinator와 End-to-End 동작은
  미검증이다.
