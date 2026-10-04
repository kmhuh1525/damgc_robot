# 프로젝트 문서

이 폴더는 `damgc_robot`의 계획, 현재 상태, 시스템 인터페이스와 시험 근거를
구분해 관리합니다. 계획에 적혀 있다는 이유만으로 구현 완료로 간주하지 않습니다.

## 먼저 읽을 문서

현재 Follower 협동 운반 상태는
[2026-10-04 최종 진행 기록](FOLLOWER_COOPERATIVE_TRANSPORT_PROGRESS_2026-10-04.md)을 먼저 읽는다.
실제 READY/예약 출발/ARRIVED/DONE 관측, heartbeat loss 이력, 실행 설정과 남은 작업을 담았다.

1. [개발 계획서](Plan.md) — 2026년 7월 13일~9월 14일의 목표, 역할, 9주 일정과 평가 기준
2. [개발 현황 및 로드맵](STATUS_AND_ROADMAP.md) — 2026년 9월 14일까지 반영한 구현 상태, 주차 게이트와 우선순위
3. [Visual SLAM 준비 및 검증 절차](VISUAL_SLAM_SETUP.md) — 현재 입력·TF 점검, rosbag 기록과 SLAM 연동 순서
4. [프로젝트 개요](PROJECT_OVERVIEW.md) — 현재 저장소 구조, 패키지 역할과 실행 방법
5. [리더·팔로워 구조](LEADER_FOLLOWER_ARCHITECTURE.md) — 목표 아키텍처와 현재 연결 상태
6. [Orin–STM32 UART 통신 규격](STM32_UART_PROTOCOL.md) — 명령, IMU, 엔코더, 상태 패킷 초안
7. [통합 진행 기록](INTEGRATION_STATUS_2026-08-25.md) — STM32·EKF·VSLAM·nvblox 실행 결과와 재현 절차
8. [VSLAM 키보드 매핑 진행 기록](VSLAM_MAPPING_PROGRESS_2026-08-28.md) — 원클릭 실행, 방향키 주행, 영구 Docker 이미지와 실제 통합 검증
9. [협동 이동 실행 가이드](COOPERATIVE_TRANSPORT_RUN_GUIDE.md) — 역할별 원클릭 실행, DDS heartbeat 확인과 fail-closed 주행 절차
10. [Survivor Stage 4 map transform 검증](SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE4_MAP_TRANSFORM_VALIDATION.md) — exact timestamp TF2, `map_positions`, 정지·이동 및 회귀 검증
11. [Survivor Stage 6 persistent registry 검증](SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_PERSISTENT_REGISTRY_VALIDATION.md) — 당시 Stage 6 구현·단일 인물 검증의 역사적 기록
12. [Survivor Stage 6.1 Registry 안정성 검증](SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_1_REGISTRY_ROBUSTNESS_VALIDATION.md) — temporal policy, multi-person physical validation, LOST/reassociation 최종 결과
13. [Survivor Stage 6.1 Registry 가시성 개선](SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE6_1_REGISTRY_VISUALIZATION_IMPROVEMENT.md) — text Z offset, VISIBLE/LOST 색상, persistent marker 자동·수동 검증
14. [Survivor Pipeline 통합 launch 개발·검증](SURVIVOR_PIPELINE_INTEGRATED_LAUNCH_VALIDATION.md) — 3-terminal 실행, ownership, 자동 검증과 실물 회귀 체크리스트

## 문서 목록

- [1차 구현·시험 기록](progress/week%201/README.md) — URDF, D435, AprilTag와 접근 상태 판정의 실행 근거
- [GitHub 등록 및 운영 기록](GITHUB_SETUP.md) — 저장소 연결과 안전한 변경 등록 절차
- [Survivor Stage 3 구현](../src/leader/rescue_robot_survivor/docs/STAGE3_CAMERA_XYZ_IMPLEMENTATION.md) — CameraInfo.P 기반 camera optical XYZ 설계와 interface
- [Survivor Stage 3 검증](../src/leader/rescue_robot_survivor/docs/STAGE3_CAMERA_XYZ_VALIDATION.md) — Docker/D435 실행 명령과 hardware test 표
- [Survivor package README](../src/leader/rescue_robot_survivor/README.md) — Stage 1~6 현재 interface와 실행 방법

팔로워의 AprilTag 접근 상태 노드 상세 명세와 시험 기록은
`src/follower/follower_supply_perception/docs/`에서 관리합니다.
현재 hybrid 알고리즘, 0.30 s tag-loss grace, freshness 분리와 ALIGNED session reset은
[`FOLLOWER_HYBRID_ALIGNMENT_MIGRATION.md`](../src/follower/follower_supply_perception/docs/FOLLOWER_HYBRID_ALIGNMENT_MIGRATION.md)를,
전체 runtime 파라미터는
[`FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md`](../src/follower/follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를 기준으로 합니다.

## 문서 기준

문서가 충돌하면 다음 순서로 해석합니다.

1. 최종 목표·범위·일정: `Plan.md`
2. 완료 여부·현재 우선순위: `STATUS_AND_ROADMAP.md`
3. 실제 ROS 이름·파라미터: 현재 소스 코드와 패키지 README
4. 특정 장비에서 수행한 결과: `docs/progress/`와 패키지별 시험 기록

상태 문서는 기능이 검증될 때 갱신하고, 실험 기록의 과거 명령과 결과는 임의로
현재 사실처럼 바꾸지 않습니다. Stage 4의 현재 판정은 전용 검증 문서와
`STATUS_AND_ROADMAP.md`, package README를 함께 기준으로 합니다.
