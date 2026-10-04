# 협동 경로 추종 개발 진행상황

작성일: 2026-10-04
범위: Leader의 Nav2 경로를 두 로봇의 형상 제약에 맞게 변환하고, Follower가 승인된 경로를 추종하도록 만드는 작업.

## 현재 상태

| 기능 | 상태 | 확인된 범위 |
|---|---|---|
| 물체·힌지 기하 모델 | 구현 | axle-hinge-contact 길이와 ±15° 수동 힌지를 반영. 실측 링크 각도와 마찰 동역학은 미대조 |
| Leader 경로 adapter | 구현 | `/plan`을 Leader axle 경로로 받아 물체 중심 경로를 역산하고 일관성 검증 |
| Formation 경로 변환 | 구현 | 물체 경로에서 Leader/Follower axle 경로와 진행 방향 계산 |
| Follower 경로 제어기 | 구현 | pure pursuit, reverse 명령, 곡률 속도 제한, TF frame 변환, stale/error/goal stop |
| 기존 안전 명령 흐름 연결 | 구현 | mission executor → selector `COOPERATION` → velocity guard → STM32 bridge |
| Follower 단독 직선 실차 추종 | 확인 | 25 cm 목표 후진 경로에서 odom 이동 약 21.6 cm, heading 변화 약 0.3°; 목표 허용오차로 자동 정지, guard off와 safe command 0 확인 |
| 가상 Leader plan → formation → Follower 실차 추종 | 확인 | 격리된 `/virtual/leader/plan`으로 11 pose 경로 입력; Leader/Follower axle 변환과 reverse 판정 확인; Follower 이동 약 21.3 cm 후 goal stop |
| 실제 Leader Nav2 `/plan` → Follower ROS 그래프 | 미검증 | 합성 경로 adapter 확인 이력은 있지만 두 Orin의 실제 Nav2 path/TF 조합은 아직 미확인 |
| RViz 격자 경로 표시 | 확인 | 가상 Leader path, Follower preview, Leader 목적지 표시. 실제 `/map`은 아직 연결 안 함 |
| 한계 곡률 S자 formation 변환 | 거부 확인 | 물체 곡률 약 `0.65 1/m` 입력에서 lateral motion ratio `0.10401`로 기준 `0.005` 초과. Leader 경로만 표시하고 Follower preview는 비움 |
| Follower 곡선 경로 실차 추종 | 확인 | 단독 reverse curve, 목표 path 길이 35 cm·반경 0.8 m; odom 실제 chord 약 31.6 cm, heading 변화 +17.5°, goal tolerance 정지 |
| 협동 미션 승인·자동 시작 | 미구현 | Follower 경로 서비스는 local `IDLE` 조건만 검사하며 Leader 상태, 파지·리프트 ACK를 승인 조건으로 쓰지 않음 |

## 방금 수행한 실차 시험 기록

- 시험 로봇: Follower만. Leader 제어기에는 명령을 보내지 않았다.
- 사용한 경로: 시험 노드가 `/cooperation/follower_path_preview`에 직접 발행한 11개 pose의 `odom` 프레임 후진 직선 경로. 시작점은 발행 당시 Follower odometry이고 목표는 25 cm 뒤였다.
- 명령 경로: Follower mission executor의 pure-pursuit 출력을 command selector의 `COOPERATION` 입력으로 remap하고, 기존 velocity guard 및 STM32 bridge를 재사용했다. 중복 bridge/I²C 접속은 만들지 않았다.
- 시작 pose: 약 `(-0.8023, -0.2934) m`; 종료 pose: 약 `(-0.8014, -0.5098) m`.
- 관측 이동: `dx≈-0.0009 m`, `dy≈-0.2164 m`, yaw 변화 약 `+0.3°`. 목표 허용오차가 5 cm라 경로 끝점 전 약 21.6 cm 지점에서 도착 판정이 났다.
- 종료: selector `STOP`, velocity guard 비활성, `/follower/safe_cmd_vel`의 모든 축 0을 확인했다. 시험용 selector와 mission executor를 종료했고, 기존 임시 bridge/guard 프로세스는 남겼다.
- launch 종료 시 mission executor가 `destroy_node()` 중 `KeyboardInterrupt` traceback으로 비정상 종료했다. 그 뒤 guard disable과 safe zero를 다시 확인했지만, 정상 종료 경로는 별도 정리 항목이다.
- 결론: Follower 경로 제어 출력에서 실제 바퀴까지 이어지는 reverse straight 경로를 확인했다. Leader가 만든 경로의 자동 수신·실행을 검증한 시험은 아니다.

### 가상 Leader `/plan` 변환을 거친 실차 시험

- 실제 Leader `/plan`과 충돌하지 않도록 입력 토픽을 `/virtual/leader/plan`으로 격리했다. 경로는 `odom` frame의 직선 25 cm, 11 pose이며, 시작 pose는 현재 Follower 위치에 생성된 Follower axle 경로가 맞도록 배치했다.
- Adapter가 11 pose를 받아 object path로 역산했다. status: `max_curvature=0.0000 1/m`, `max_hinge=0.00°`, `leader_drive=FORWARD`, `follower_drive=REVERSE`.
- Formation 변환은 11 pose를 수락했고 `curvature_limit=0.6795 1/m`, `max_lateral_ratio=0`, Follower 방향 `REVERSE`를 보고했다.
- Preview 제어 출력은 `v=-0.080 m/s`, `ω≈0.00027 rad/s`, `cross_track=0.000 m`였다. preview 토픽은 모터 명령과 분리되어 있었다.
- 이어 같은 Follower 경로를 mission executor의 guarded motor 경로로 보냈다. 시작 pose `(-0.801407, -0.509663) m`, 종료 pose `(-0.799613, -0.722754) m`, 이동 `(+0.001794, -0.213091) m`, yaw 변화 약 `+0.08°`였다. 목표 25 cm와 goal tolerance 5 cm로 약 21.3 cm에서 goal stop이 났다.
- Selector가 `STOP`으로 돌아왔고 guard 비활성화 후 `/follower/safe_cmd_vel=0`을 다시 확인했다. 기존 `/plan`이나 Leader 제어에는 입력을 보내지 않았다.
- 이 시험은 가상 Leader plan의 **전체 수학적 변환과 Follower 실차 추종**은 확인했지만, 실제 Leader Nav2 publisher/DDS/frame/TF 연결은 여전히 미검증이다. 직선 경로였으므로 곡선 변환·곡선 주행 검증도 아니다.

### Follower 단독 곡선 추종 시험

- 가상 Leader나 물체 입력 없이 `/cooperation/follower_path_preview`로 Follower의 `odom` 프레임 곡선 경로를 직접 넣었다. 목표 경로 길이 0.35 m, 반경 0.8 m의 후진 호였고, 시작점은 당시 Follower pose였다.
- 추종기 목표 도달 판정 후 selector `STOP`으로 전환됐다. 시작 `(-0.799613, -0.722754) m`, 종료 `(-0.739515, -1.032603) m`; odom 변화 `(+0.060098, -0.309849) m`, 위치 간 chord 약 0.316 m, yaw 변화 `+17.47°`였다.
- guard 비활성화와 `/follower/safe_cmd_vel` 0을 확인했다. Leader와 두 로봇 협동은 실행하지 않았다.
- 이 시험은 단독 Follower pure-pursuit의 reverse curve 기본 동작을 확인했다. 경로 추종의 정확한 곡률 편차, 힌지에 연결된 물체의 곡선 운반, 곡선 장애물 회피는 검증하지 않았다.

### 격자 RViz 및 한계 곡률 S자 미리보기

- RViz top-down 격자 보기를 띄우고, 지도 표시를 끈 상태에서 경로와 목적지 marker가 표시되는 것을 확인했다.
- 가상 리더 경로 입력은 실제 Leader `/plan`과 분리된 `/virtual/leader/plan`을 사용했다. Leader/Nav2에는 명령을 보내지 않았고, 로봇 구동 명령도 발행하지 않았다.
- 물체 경로 곡률 `0.65 1/m`(설정 한계 `0.6795 1/m`에 근접)의 S자를 생성해 Leader axle 경로로 변환하고 Leader adapter에 넣었다. Leader 경로의 원시 시각화와 목적지는 표시됐다.
- 역산 뒤 Formation 검증에서 lateral motion ratio `0.10401`이 한계 `0.005`를 넘어서 Follower 경로를 거부했다. 거부된 입력에서 이전 Follower 경로가 화면에 남지 않도록 empty path도 발행한다.
- 이 결과는 한계 곡률의 일정 곡률 원호가 통과할 수 있어도, 곡률 부호가 바뀌는 급한 S자 경로까지 통과한다는 뜻은 아님을 보여준다. 가상 S자 rejection은 로봇 구동 시험이 아니다.

## 확정된 기구·제어 설정

- 두 로봇은 동일 형상이고 물체의 서로 반대편을 잡는다. 파지 TCP 기준 axle 중심에서 물체 중심까지 31 cm, 상자 두께 10.5 cm이므로 axle에서 접촉면까지 25.75 cm다.
- 힌지 중심은 axle에서 12.5 cm, hinge에서 접촉점까지 13.25 cm, 상자 중심에서 접촉면까지 5.25 cm로 모델링한다.
- 수동 yaw 힌지는 중립 기준 ±15°다. 계획기에는 20% 여유를 두어 ±12°를 사용한다. 현재 모델의 최소 물체 경로 반경은 약 1.47 m다.
- Follower `base_link`는 바퀴축 중점, wheel separation은 23 cm로 둔다. 실차 추종은 휠 odometry `/follower/odom/raw`를 사용한다.
- 기본 pure-pursuit lookahead 0.20 m, 시작/추종 최대 횡오차 0.20 m, goal tolerance 0.05 m, 선속도 상한 0.08 m/s, 각속도 상한 0.30 rad/s, lateral acceleration 0.08 m/s²다.
- 별도 시험 동안 관측한 Follower odometry 발행률은 약 46 Hz였다. 추종 시 stale 기준은 0.35 s다.

## 실행 선행 조건

1. 두 Orin에서 동일한 ROS 2 domain과 호환되는 DDS 설정을 사용하고, Follower가 Leader Nav2의 `/plan`을 실제로 받아야 한다.
2. `/plan`의 `header.frame_id`와 pose 방향이 유효해야 한다. pose orientation은 경로 진행 방향을 향해야 한다.
3. Follower `/follower/odom/raw`가 신선해야 한다. 경로 frame과 odometry frame이 다르면 TF tree에 `path_frame ← odom_frame` 변환이 있어야 한다.
4. `/cooperation/follower_path_preview`가 유효한 pose 두 개 이상을 가져야 하고, 선택한 시작점에서 Follower까지 오차가 0.20 m 이하이어야 한다.
5. `/follower/mission/cmd_vel`에 mission executor 하나만 발행해야 한다. selector의 COOPERATION 입력 remap, velocity guard, bridge가 단일 연결이어야 한다.
6. 경로 방향은 한 경로 안에서 전진/후진이 섞이지 않아야 한다. 현재 추종기는 정지 후 기어 방향이 바뀌는 cusp 경로를 이어서 수행하지 않는다.
7. 실제 시작은 현재 서비스 `/follower/path_tracking/enable`로 수동 승인한다. 경로 수신만으로 구동하지 않는다.

## 협동 자동 실행 전에 추가로 정해야 할 인터페이스

독립 단독 시험 서비스를 협동 운반에 그대로 쓰면 안 된다. 이 서비스는 Follower가 `IDLE`인지만 확인하고 Leader가 바쁘거나 물체를 잡지 않은 상태인지 판단하지 않는다. 자동 협동 실행을 붙일 때 다음 승인 흐름을 구현해야 한다.

1. Leader mission coordinator가 Nav2 goal과 유효한 최신 `/plan`을 식별하고, plan의 timestamp/frame/path ID를 Follower에 전달한다.
2. Follower adapter가 그 plan을 받아 상자 중심 경로 역산, hinge/차동구동 feasibility, Follower axle 경로 변환을 완료한다.
3. Follower가 `PATH_READY` 또는 명시적 reject 이유를 Leader에 돌려준다. Leader는 양쪽 로봇이 grasp 완료, lift 완료, 경로 승인 상태인지 확인한다.
4. Leader가 명시적 `START_PATH(path_id)` 승인을 보낸 뒤에만 Follower selector를 COOPERATION으로 바꾸고 guard를 활성화한다. 서비스는 계속 단독 점검용으로 남기거나 권한 있는 action으로 대체한다.
5. 추종 중 cancel/abort, Leader heartbeat 손실, plan 교체, Follower stale odom, 경로 오차/hinge feasibility 상실이 생기면 양쪽 로봇이 합의된 정지 절차를 실행한다.
6. Follower `PATH_DONE` 또는 `PATH_FAULT(reason)`을 Leader에 전달한다. 새 plan을 적용할 때는 기존 경로 추종을 먼저 정지하고 새 경로 검증 및 재승인을 받아야 한다.

아직 확정되지 않은 정책은 Leader mission의 path owner, path ID 생성 규칙, plan 갱신 중 처리, 한 로봇의 통신 단절 시 물체 하중을 유지하는 방법이다. 이 항목들은 협동운반을 실제 실행하기 전에 결정해야 한다.

## 다음 개발 순서

### 1. 실제 Leader 계획 경로의 무구동 통합 확인

- Follower 기본 launch에서 자동으로 시작하는 Leader path adapter와 formation 변환기를 켠다.
- Leader Nav2가 publish한 실제 `/plan`이 Follower 네트워크로 도착하는지, frame, 시간, path pose 방향을 기록한다.
- RViz에서 Leader axle·Follower axle 경로를 확인하고 status가 feasible/rejected인 이유를 기록한다.
- 이 단계에서는 `/follower/path_tracking/enable`을 호출하지 않고 guard를 끈 상태로 둔다.

### 2. Follower 단독 곡선 정밀도와 정지 동작 확인

- 이미 reverse curve 기본 추종은 확인했다. 후속 시험에서는 계획 경로와 odometry의 cross-track 편차, 실제 반경, 양쪽 바퀴 응답과 guard 출력 속도를 기록한다.
- goal stop, 수동 stop, stale odometry, 큰 cross-track, TF 누락 각각이 selector STOP과 safe zero로 끝나는지 확인한다.
- emergency stop은 기존 하드웨어 절차로 별도 확인한다. 소프트웨어 stop 결과로 대체하지 않는다.

### 3. 협동 mission 승인 프로토콜 연결

- `PATH_READY`, `START_PATH`, `PATH_DONE`, `PATH_FAULT`, `CANCEL_PATH`의 payload, session/sequence, timeout, 중복 메시지 처리를 정의한다.
- Leader mission state machine에 경로 준비/승인/추종/완료 상태를 추가한다.
- Follower가 grasp/lift 전이 중이거나 다른 제어기가 명령을 발행할 때 경로 추종이 시작되지 않는지 확인한다.
- 두 로봇을 들지 않은 상태의 저속/짧은 협동 이동, abort와 링크 단절 후 정지 시험을 통과한 뒤에만 물체를 든다.

## 현재 ROS 운영 상태

2026-10-04 현재 임시 `/tmp/follower_solo_drive.launch.py`의 Follower STM32 bridge와 velocity guard는 떠 있고, guard는 비활성이다. 고곡률 S자 미리보기용 가상 plan publisher와 adapter/formation preview, RViz는 표시 확인을 위해 실행 중이며 `/virtual/leader/plan`만 사용한다. 경로 미리보기 노드는 velocity 명령을 발행하지 않는다. 기본 follower mission launch로 전환하려면 기존 bridge/guard 중복 기동을 피하도록 현재 임시 launch를 먼저 정리해야 한다.
