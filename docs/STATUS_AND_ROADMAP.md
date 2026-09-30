# 개발 현황 및 로드맵

## 문서 목적

이 문서는 [개발 계획서](Plan.md)를 실행 상태로 변환한 관리 문서다. 최초 기준일은
**2026년 7월 27일**이며, 현재 상태는 2026년 9월 18일까지 반영한다. AprilTag software
pipeline, Survivor Stage 1·3·4·5 실기 검증과 Stage 2 구현 상태를 아래
관련 행에 반영했다.

상태 표시는 다음 기준을 사용한다.

- **완료**: 저장소 구현과 시험 기록으로 확인됨
- **부분 완료**: 하위 기능은 있으나 계획의 종료 조건을 충족하지 못함
- **미구현**: 저장소에서 구현을 확인할 수 없음
- **확인 필요**: 기구·전장·다른 장비 작업처럼 이 저장소만으로 판단할 수 없음

## 목표 대비 현재 상태

| 계획의 세부 목표 | 상태 | 현재 근거 또는 부족한 조건 |
| --- | --- | --- |
| D435 실시간 3차원 지도 | 완료(범위 내) | single-owner D435, VSLAM, dual EKF, nvblox mesh/3D ESDF와 RViz를 Jetson에서 동시 검증; 장기 주행 안정성은 계속 관찰 |
| Visual SLAM 위치·자세 추정 | 완료(범위 내) | infra1/infra2, `/visual_slam/tracking/odometry`, status Success와 저속 A→B 이동 검증 |
| BNO055와 wheel odometry 보정 | 부분 완료 | 현재 장비는 STM32 IMU/wheel odometry와 dual EKF를 검증; BNO055 기반 계획 항목은 별도 미완료 |
| 카메라 기반 생존자 탐지 | 완료 | survivor 전용 Docker GPU runtime과 YOLO11n pipeline 검증; 실제 D435에서 1/2/3명 검출, bbox/confidence, 좌→우 numbering 및 rqt debug image 확인 |
| depth 기반 생존자 3차원 위치 | 완료(범위 내) | Stage 3 camera optical XYZ와 Stage 4 exact-timestamp TF2 map XYZ, `/leader/survivor/map_positions`를 실제 D435에서 검증 |
| Nav2 자율주행 | 미구현 | Nav2 구성·지도·주행 시험 없음 |
| AprilTag 물품 인식·정밀 접근 | 부분 완료 | 양 로봇 camera/base alignment와 guarded software velocity 구현; 실제 접근·파지 검증 필요 |
| 그리퍼 물품 파지 | 부분 완료 | Dynamixel RX-28/RX-64 노드와 `cooperative_mission`의 ALIGNED→닫기·동시 리프트 구현(2026-09-30); 실물 파지 시험 필요 |
| 경량 물품 단독 운반 | 미구현 | 접근·파지·주행 연결 없음 |
| 중량 물품 협동 운반 | 부분 완료 | `cooperative_mission`: 리더 파지 → 팔로워 반대편 파지 → ACK 기반 동시 리프트 → 공통 속도(팔로워 부호 반전) 1초 직진을 소프트웨어·시뮬레이션으로 검증(2026-09-30); 실물 시험 필요 |
| 지도·생존자·로봇 상태 시각화 | 완료(범위 내) | RViz 3D map과 Stage 5 raw marker, Stage 6.1 persistent Registry sphere/text를 Jetson에서 동시 검증; LOST yellow LAST SEEN과 white text 포함 |

## 현재 저장소에서 재현 가능한 범위

### 리더

- 실제 치수를 반영한 URDF와 RViz 표시
- D435 RGB·depth 토픽 발행
- RGB 보정과 CameraInfo QoS 보조
- `tag36h11`, ID 0, 0.050 m 기준 AprilTag 검출
- depth 영상 중앙 20×20 영역의 거리 CSV 저장
- COCO person 다중 검출, 좌우 frame-local 번호와 `/leader/survivor/debug_image` 구현 및
  실제 D435 1/2/3명 hardware verification 완료 (`VERIFIED`, 2026-09-11)

Stage 2는 YOLO bounding box + aligned depth의 중앙 ROI median 거리[m]를 계산한다. Stage
3는 같은 ROI center와 median Z, rectified CameraInfo.P로 camera optical XYZ를 계산하고
`/leader/survivor/camera_positions` PoseArray를 발행한다. Stage 3는 실제 사람 좌/우 X 부호,
distance/Z 일치, debug XYZ, 다중 사람 pose, `camera_color_optical_frame`과 identity quaternion을
확인해 `VERIFIED (2026-09-12)`다. Stage 4는 `survivor_map_transform_node`에서
`Time.from_msg(msg.header.stamp)`를 사용해 `/leader/survivor/map_positions`를 생성하고,
TF lookup 실패 시 발행하지 않는 정책까지 실제 검증했다. 정지 255 valid pairs와 A→B
약 0.351 m 이동에서 camera XYZ 변화 및 map XYZ의 생존자 주변 유지가 확인됐다. Stage 2의
별도 정식 줄자 거리표와 Stage 4 raw map의 약 0.115 m A→B 변화는 제한사항으로 남아 있다.
Stage 5의 `/leader/survivor/map_markers`와 RViz sphere/text, nvblox mesh 동시
표시, 통제된 2명→1명 marker 제거까지 수동 검증했다. 후보 번호는 현재 배열
인덱스일 뿐 persistent ID가 아니다. 다음 survivor 단계는 Stage 6 persistent
Survivor ID, spatial deduplication, position stabilization이다.

### 팔로워

- USB 카메라 보정과 AprilTag 검출
- `follower_camera_optical_frame` 기준 태그 상대 pose
- 거리·좌우 오차·수평각 계산
- TF timeout, 중앙값 필터와 9개 접근 상태
- ROS 비의존 단위 시험과 전체/상태 전용 launch
- exact-stamp `base_link` hybrid alignment와 atomic alignment command
- approach controller, command selector, final velocity guard와 `/follower/safe_cmd_vel`
- `base_stable_time=0.30 s`, fresh confirmation 3회와 ALIGNED session latch/reset
- FINAL_APPROACH/STABILIZING 0.30 s tag-loss grace; tag 미검출 중 velocity zero
- source stamp/monotonic receipt freshness 분리와 blind-final 기본 OFF

이 범위는 software command pipeline까지다. 실제 모터 주행, 그리퍼 또는 협동 운반이
검증됐다는 의미가 아니다.

## 주차 게이트 점검

### 1주차 게이트: 인터페이스와 개발환경 고정

| 종료 조건 | 상태 | 조치 |
| --- | --- | --- |
| 두 Orin에서 같은 workspace 빌드 | 확인 필요 | 두 장비의 커밋·의존성·빌드 결과 기록 |
| `/leader`, `/follower` namespace 통신 | 부분 완료 | 개별 토픽은 사용, 두 Orin 네트워크 시험 필요 |
| 차체–카메라–IMU TF 확인 | 부분 완료 | 리더 `map → odom → base_link → camera_color_optical_frame`과 IMU/wheel 경로는 검증; 팔로워 차체 TF는 별도 |
| RGB-D, IMU, 모터 데이터 확인 | 부분 완료 | 리더 RGB-D, STM32 IMU/wheel 데이터는 검증; 양 로봇 전체 시험은 별도 |
| STM32 패킷·시간 동기화·Mission 상태도 | 미구현 | 공통 인터페이스 문서와 가짜 노드 필요 |
| 차체·그리퍼·부품 확정 | 확인 필요 | 기구/BOM 담당 기록 연결 필요 |

1주차 게이트가 닫히지 않았으므로 네임스페이스, TF frame, STM32 패킷, 비상정지
경로를 3주차 기능과 병행해 먼저 고정해야 한다.

### 2주차 게이트: 로봇 A 주행과 독립 기능

| 종료 조건 | 상태 | 조치 |
| --- | --- | --- |
| 로봇 A 원격 직진·회전 | 확인 필요 | `/leader/cmd_vel`부터 STM32까지 실물 시험 |
| 3차원 지도 5분 이상 생성 | 완료(범위 내) | 469.0초 rosbag에서 VSLAM, nvblox mesh/3D ESDF, RViz를 동시 검증 |
| 사람과 AprilTag 검출 | 완료 | AprilTag와 Survivor Stage 1 완료; 사람 탐지는 실제 Jetson/D435에서 검증됨 |
| 규격 물품 수동 파지 | 확인 필요 | 그리퍼 시험 기록 필요 |
| 가짜 노드로 임무 상태 전체 순환 | 완료(소프트웨어) | `cooperative_mission` 상태기계와 `test/sim_world.py` 가상 로봇·링크 시험(정상/단절/실패/재전송/release) |

### 3주차 게이트: 로봇 A 자율주행과 로봇 B 기본 구동

| 종료 조건 | 상태 | 이번 주 산출물 |
| --- | --- | --- |
| 로봇 A 목표점 반복 이동 | 미구현 | 정적 지도 Nav2 최소 구성과 반복 시험 |
| 사람 위치를 카메라·map 좌표로 출력 | 완료(범위 내) | Stage 3 camera XYZ와 Stage 4 exact-timestamp `map_positions`, A→B 이동 검증 완료 |
| 로봇 B 원격 속도 주행 | 미구현/확인 필요 | STM32 통신과 `/follower/cmd_vel` 연결 |
| 두 로봇 긴급정지 | 미구현/확인 필요 | 하드웨어 E-stop과 소프트웨어 정지 경로 검증 |
| 전원·발열 30분 시험 | 확인 필요 | 전압·온도·재부팅 여부 기록 |

## 우선순위

### P0 — 통합 전에 반드시 고정

1. 두 Orin의 커밋, ROS 환경, 의존성, ROS domain과 시간 동기화 확인
2. `/leader`, `/follower` 토픽 규칙과 양쪽 TF frame 충돌 방지 규칙 확정
3. `base_link`에서 실제 camera optical frame과 `imu_link`까지의 TF 측정
4. Orin–STM32 패킷, 50 Hz 명령·상태, timeout과 비상정지 동작 정의
5. Mission Coordinator 상태와 가짜 토픽 시험 정의

### P1 — 3주차 종료 조건

1. wheel odometry와 BNO055를 `robot_localization`에 연결
2. 로봇 A의 정적 지도 기반 Nav2 목표점 반복 이동
3. Stage 4에서 검증된 exact-timestamp TF2 map XYZ를 기반으로 RViz Marker를 구현
4. 로봇 B의 `/follower/cmd_vel` 주행
5. 두 로봇 E-stop과 전원·발열 30분 시험

### P2 — 이미 앞서 구현된 기능의 현장 검증

1. 태그를 좌우·전후로 이동해 팔로워 hybrid 상태와 부호 확인
2. FINAL_APPROACH/STABILIZING 0.30 s grace, zero velocity와 재검출 복구 실측
3. 실제 물품과 그리퍼 형상으로 목표 거리·허용 오차 재측정
4. 4주차 저속 정렬 제어에 사용할 안전 제한과 watchdog 확정

## 계획 인터페이스와 현재 구현의 관계

계획의 로봇별 odometry·IMU와 실제 배터리·fault 구조화 상태는 아직 목표
인터페이스다. 리더 저장소에는 `leader_cooperation` 패키지가 추가되어
`/follower/cmd_vel`, `/cooperation/*`, `/mission/state`의 리더 측 발행과
`/follower/status` heartbeat 구독을 제공한다.

메시지 타입, QoS, 발행 주기, timeout, 담당 노드와 fault 동작이 확정되기 전까지
목표 인터페이스를 구현 완료로 표시하지 않는다. 자세한 구분은
[리더·팔로워 구조](LEADER_FOLLOWER_ARCHITECTURE.md)를 따른다.

## 갱신 규칙

- 기능 상태를 바꿀 때 실행 명령, 날짜, 장비와 결과 문서 링크를 함께 남긴다.
- “노드가 실행됨”과 “계획의 종료 조건을 만족함”을 구분한다.
- 하드웨어 시험을 하지 않은 기능은 코드가 있어도 부분 완료로 표시한다.
- 매주 종료일에 해당 주차 게이트와 다음 주 우선순위를 갱신한다.
- 8월 30일 이후에는 계획대로 신규 기능을 추가하지 않고 안정화 기록만 갱신한다.
