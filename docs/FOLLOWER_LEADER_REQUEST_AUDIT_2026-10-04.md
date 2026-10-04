# Leader 문서의 Follower 요구사항 대조

확인일: 2026-10-04. GitHub의 두 브랜치(main, codex/cooperative-mission-20260930)와
현재 작업 트리를 읽어 대조했다. 구현 여부는 소스 확인 결과이며, 이번 확인에서 노드를
실행하거나 모터를 구동하지 않았다. 새 peer workflow의 양쪽 실차 통합은 미검증이다.

## 확인한 문서

- [Leader 협동 DDS README](../src/leader/leader_cooperation/README.md)
- [Leader/Follower 구조](LEADER_FOLLOWER_ARCHITECTURE.md)
- [Leader 통합 Phase 1–2](LEADER_INTEGRATION_PHASE1_2_2026-10-02.md)
- [Leader 시나리오 계획](LEADER_SCENARIO_V1_DEVELOPMENT_PLAN.md)
- [Leader 그리퍼 통합의 Follower 적용값](../src/leader/rescue_robot_apriltag/docs/LEADER_FINAL_APPROACH_TAG_LOSS_AND_GRIPPER_INTEGRATION_GUIDE.md)
- [협동 미션 README](../src/cooperative_mission/README.md)
- 로컬 미커밋 `src/cooperative_transport/README.md`의 새 수동 파지 운반 계약

문서 작성자의 역할은 Git 기록만으로 단정하지 않는다. 위 문서에 적힌 Follower 요구와
현재 코드를 비교했다. 기존 속도 전달/자동 파지 미션과 새 수동 파지 peer 운반은 서로
다른 실행 흐름이다. 새 흐름에서는 AprilTag 자동 정렬·그리퍼·자동 lift를 실행하지 않는다.

## 요구별 결과

| Follower 요구 | 확인 결과 | 남은 확인 |
|---|---|---|
| 같은 ROS domain/RMW로 DDS 통신 | 실행 환경 계약 있음. 이전 가이드 domain 42, 새 peer 예시 domain 0 | 두 Orin 실제 실행 환경과 discovery 일치 확인 |
| 기존 `/follower/status` String heartbeat | 기존 velocity guard가 주기적으로 발행 | Leader 실제 수신 확인 |
| 기존 `/follower/cmd_vel` 명령 수신 | 기본 selector의 COOPERATION 입력 존재 | 새 peer launch에서는 `/follower/mission/cmd_vel`로 remap됨. 기존 DDS 전달 launch와 혼용 금지 |
| mission 명령 session/seq, ACK, 중복 실행 방지 | 기존 follower mission logic/node에 구현 | 실제 두 Orin 자동 파지/lift 미션 확인 |
| 태그 인식·정렬 및 파지·lift 요청 수행 | 기존 mission state machine과 서비스/그리퍼 연결 존재 | 새 peer 범위에는 없음. 양쪽 결합 실차 미검증 |
| 새 운반 launch는 peer·selector·guard·STM32만 실행 | 로컬 manual_transport launch에 구현. 그리퍼 launch 없음 | 패키지 커밋·빌드·배포, 기존 노드 중복 해소 |
| selector STOP, guard disabled로 시작 | 새 launch와 기존 guard 인자에 구현 | 실행 중 startup 상태와 단일 motor publisher 확인 |
| `/follower/odom/raw` 및 자기 로컬 frame 사용 | bridge 출력과 peer 구독 일치. PREPARE에서 한 세션 SE(2) 변환 | 실제 장착은 중립 힌지·반대 방향·차축 62 cm여야 함. 상대 pose를 센서로 측정하는 기능은 아님 |
| 경로 hash/geometry 독립 검증 후 READY | 로컬 peer에 hash 확인·formation 재계산·geometry 대조·selector STOP 응답 확인 구현 | 실제 Leader packet 상호 수신 및 reject 확인 |
| N 전까지 정지, ARM/ACK와 COMMIT/ACK 후 예약 시작 | 로컬 peer에 구현. RTT 200 ms 이하, 시작 약 2초 후, deadline 확인 | 실제 시작 시차 측정. 소프트웨어 예약은 동시 시작 보장이 아님 |
| Leader 전진 시 Follower 후진, 속도/진행도 연동 | 로컬 peer의 reverse-capable pure pursuit와 Leader speed ratio 적용 | 기존 단독 추종 실차 기록은 있음. 새 peer의 두 로봇 결합 추종은 미검증 |
| heartbeat/odom 끊김·경로 오차·충돌 publisher·ABORT 정지 | 로컬 peer에 zero, selector STOP, guard disable 요청 구현 | 실제 종료 서비스 응답·연결 단절 정지 확인 |
| Follower 경로/status RViz 표시 | 새 peer가 transient-local Path/status 발행 | 각 로컬 frame 사용. 같은 `odom` 이름이어도 원점이 같다는 뜻은 아님 |

## 발견한 계약 불일치

1. **B/N 요청 중계:** `transport_keys`는 `/leader/mapping/control`에
   `COOP_PREPARE/COOP_START/COOP_ABORT`를 발행한다. peer는
   `/cooperation/transport/control`의 `PREPARE/START/ABORT`를 구독한다.
   현재 확인한 Leader 소스에는 이 변환 중계를 찾지 못했다. 새 README가 설명하는
   업데이트된 mapping/keyboard 연결이 실제 Leader 브랜치에 반영됐는지 확인해야 한다.
2. **Leader selector:** peer가 요구하는 `COOPERATION` source와
   `enable_nav2_goal_selection` 파라미터가 현재 공용 Leader selector에는 없다.
   Follower selector의 COOPERATION 지원과는 별개다. Leader 미반영 상태에서는
   PREPARE/START 성공을 전제로 Follower 완료라고 보고하면 안 된다.
3. **그리퍼 값 문서 정합성 수정 완료:** Leader 그리퍼 가이드의 Follower 부분을 현재
   드라이버 프로필·launch·mission 설정에 맞춰 RX-64 `20..270`, LIFT `20`,
   LOWER `270`으로 수정했다. RX-28 ID `1`, OPEN `950`, CLOSE `350`, RX-64 ID
   `50`, Moving Speed `50`을 사용한다. 수동 파지 peer에는 그리퍼 명령을 연결하지 않는다.
4. **코드 공유 상태:** 새 `src/cooperative_transport`는 확인 시점에 untracked다.
   이 문서가 GitHub에 올라가도 새 peer 코드가 배포됐다는 뜻은 아니다.
5. **기존 인계 메모 보완:** [Leader 인계 메모](COOPERATIVE_TRANSPORT_LEADER_HANDOFF.md)의
   mapping/control 설명도 실제 발행·구독 토픽과 중계 요구에 맞춰 수정했다. peer의
   제어 입력은 `/cooperation/transport/control`이다.

Follower의 기존 단독 경로 생성/추종 결과는
[경로 추종 진행상황](COOPERATIVE_PATH_TRACKING_PROGRESS.md)에 있다.
새 수동 파지 peer는 코드 준비와 실제 통합 확인을 구분해서 관리해야 한다.
