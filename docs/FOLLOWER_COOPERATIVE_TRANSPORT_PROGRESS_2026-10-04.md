# Follower 협동 운반 최종 진행 기록

기록일: 2026-10-04. 현재 작업 브랜치는 `codex/cooperative-mission-20260930`이며,
Follower 저장소는 `kmhuh1525/damgc_robot`, Leader 저장소는 `rbgusrns/damgc_robot`다.
기존 단독 경로 시험과 새 수동 합체 peer 흐름을 구분한다.

## 완료된 개발

| 항목 | 완료 범위 |
|---|---|
| 물체·힌지 기하 | axle→hinge 0.125m, hinge→contact 0.1325m, centre→contact 0.0525m, axle 간격 0.62m, 물리 힌지 ±15° |
| 기존 Follower 경로 추종 | 합성 Leader 경로 변환, 격자 RViz, 후진 직선·단독 곡선 실차 시험 기록 있음 |
| 수동 합체 peer | session/hash JSON v1, PREPARE/READY, clock ping, ARM/ACK, COMMIT/ACK, heartbeat, STOP |
| 독립 경로 확인 | Leader 경로로 formation 재계산, geometry/hash/전달 Follower 배열 비교, 로컬 odom으로 SE(2) 정렬 |
| 구동 연결 | peer → `/follower/mission/cmd_vel` → selector COOPERATION → guard → `/follower/safe_cmd_vel` → STM32 |
| 준비·출발 분리 | startup STOP/guard disabled, READY는 이동하지 않음, N 이후 예약 시각에 출력 gate 개방 |
| 정지 처리 | odom/heartbeat timeout, tracking error, conflicting publisher, 늦은 COMMIT, service timeout, abort/goal stop |
| 임시 곡률 옵션 | `validate_curvature=false`로 곡률·힌지 한계 검사 생략 가능. 계산·hash·geometry·횡이동·round-trip 검사는 유지 |
| Leader d812a04 동기화 | 반복 역산을 리더 차축 곡률의 직접 힌지 계산으로 교체. 동일 평활화 폭·5cm 보간·원 입력 body 사용 |
| 현행 그리퍼 설정 | Follower RX-64 ID50, 범위20..270, LIFT20/LOWER270; RX-28 ID1, OPEN950/CLOSE350 |

새 peer 흐름에서는 Dynamixel, AprilTag approach, 자동 grasp/lift mission을 실행하지 않는다.
현재 코드의 준비용 기본값은 이동·I2C 쓰기 false다. 이후 사용자의 실차 출발 요청으로
실행 중 Follower의 `motion_enabled`와 bridge의 `i2c_write_enabled`를 true로 변경했다.
기존 bridge를 유지하여 odom 원점을 보존하면서 peer만 교체했다.

주요 코드 공유 이력:

- `2a4b315`, `80bf30f`, `739a9d8`: 인계 계약·요구사항 대조·현재 설정 문서화
- `345b31c`: 새 peer와 기본 이동 차단·격리 ROS 검증
- `5a3233c`: 임시 곡률 검사 생략 옵션
- `2b35f3c`: Leader `d812a04` 직접 계산식과 원본 경로 대조 fixture

## 실제 네트워크와 실차 상태 관측

두 로봇 모두 ROS domain 0, localhost_only 0, FastDDS/UDPv4를 사용한다.
Follower peer·selector·guard·bridge가 각각 한 개이고 mission executor/Dynamixel은
실행하지 않았다. Follower odometry와 정지 상태의 command 0을 현장에서 확인했다.

이전 준비 과정에서 Leader가 곡률 한계, 시작 pose/heading 불일치, 횡이동 비율 초과로
경로를 거절했다. 이때 Follower는 IDLE이었으며 경로를 받거나 출발하지 않았다.
이 거절 이력은 현재 완료 상태와 구분해야 한다.

사용자가 Leader를 조작한 뒤 실제 실행 로그에서 다음 두 세션을 관측했다.
시각은 KST이며 원본은 로컬 `log/follower_drive_enabled_20261004.log`다.

| 실행 | 관측된 상태 전이 | 결과 |
|---|---|---|
| 첫 실행 | 20:12:43 READY → 20:12:44 ARMED/SCHEDULED → 20:12:46 RUNNING → 20:13:01 STOPPED | `peer heartbeat lost`로 정지 |
| 다음 실행 | 20:13:45 READY → 20:13:46 ARMED/SCHEDULED → 20:13:48 RUNNING → 20:14:08 ARRIVED → DONE | Leader의 goal 완료 통보 후 양쪽 DONE |

두 번째 실행의 핵심 원본 로그:

```text
[1791112425.518170207] COOP follower READY: path frozen; motors stopped; awaiting leader start key
[1791112426.213528539] COOP follower ARMED: selector and guard armed; command gate still zero
[1791112426.399989532] COOP follower SCHEDULED: common start accepted; zero until deadline
[1791112428.355323397] COOP follower RUNNING: scheduled gate opened; following frozen path
[1791112448.255691582] COOP follower ARRIVED: at goal; holding zero while peer finishes
[1791112448.497053625] COOP follower DONE: peer stopped: DONE
```

문서화 작업 중 최종 ROS snapshot에서 Leader `DONE: goal reached; both command gates stopped`,
Follower `DONE: peer stopped: DONE`, Follower command `(0,0)`와 odometry 실측 속도
`(0,0)`를 확인했다. 이 기록은 실제 연결에서 실행과 완료 상태 전이가 있었음을
보여준다. 경로 오차·총 이동량·물체 하중·실제 양쪽 출발 시차는 계측하지 않았으므로
정밀도나 동시 출발 정확도가 검증됐다고 해석하지 않는다.

## 검증 결과

- `cooperative_transport`, Follower selector·guard, STM32 bridge 빌드 완료.
- 새 peer/launch/곡률 옵션/Leader 기준 경로 대조: 격리 ROS domain 173에서 **22 passed**.
  실제 bridge와 모터 없이 selector·guard를 연결한 테스트다.
- 그리퍼 현재 설정 변경에 맞춰 기존 시뮬레이션의 lift 판별과 Follower lower 기대값도
  수정했다. 해당 mission·launch·Dynamixel mock 회귀는 **71 passed**다.
  이 테스트는 실제 그리퍼나 모터를 구동하지 않는다.
- 문서·코드 whitespace 확인 후 명시한 파일만 커밋한다.

## 단독 키보드 조작

사용자는 Follower 단독 키보드 조작을 요청했으나, 이어서 문서화·push로 작업을 전환했다.
키보드 코드/실행 방법 조사까지만 했으며, 키보드 주행기를 띄우거나 주행 명령을 보내지
않았다. 현재 협동 selector에는 별도의 TELEOP source가 없고, 기존 입력 소유권을
정리해서 selector/guard를 통과하는 단독 수동 실행 구성을 만드는 작업이 남았다.

## 남은 작업

1. 첫 실행의 heartbeat loss 원인과 실제 DDS 지연/손실을 계측한다.
2. 실제 계획 경로와 양쪽 odometry를 함께 기록해 경로 오차·실제 출발 시차를 산출한다.
3. 직선 후 완만한 곡선·장애물 배치에서 합체 상태 추종을 확인한다.
4. 실제 경로에 대해 곡률 검사를 다시 켤 조건을 정한다. 현재 Follower runtime의
   임시 검사 생략 설정을 합체 feasibility 검증 완료로 취급하지 않는다.
5. DONE의 STOP 재전송으로 동일 terminal 로그가 반복되는 현상을 정리한다.
6. 키보드 단독 조작은 별도 명령 owner와 종료 시 정지 동작을 갖춰 연결한다.

계산식·실행 설정의 상세는 [Leader 계산식 동기화](FOLLOWER_LEADER_D812A04_SYNC.md),
[Follower 준비 기록](FOLLOWER_MANUAL_TRANSPORT_PREPARATION.md),
[Leader 요구사항 대조](FOLLOWER_LEADER_REQUEST_AUDIT_2026-10-04.md)를 참조한다.
