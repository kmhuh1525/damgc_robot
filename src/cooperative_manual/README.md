# 터미널 수동 직선 이동·양쪽 그리퍼

실제 리더 경로를 사용하는 새 곡선 모드: [곡선 운반 인계](../../docs/MANUAL_CURVE_TRANSPORT_HANDOFF.md). 기존 직선 모드는 기본값으로 유지하며, 곡선 모드는 별도 선택·무구동 검증 단계입니다.

하나의 전용 키보드로 리더와 팔로워를 조작한다. 양쪽 노드는 같은 패키지의 role만 바꿔 실행한다.

| 키 | 동작 |
|---|---|
| e | 양쪽 준비 후 리더 활성화 |
| ↑ / w 유지 | 리더 전진·팔로워 후진 |
| ↓ / s 유지 | 리더 후진·팔로워 전진 |
| 키 해제 / x | 같은 ramp로 감속 |
| G | 양쪽 정지 확인 → 양쪽 파지 |
| R | 양쪽 정지 확인 → 양쪽 그리퍼 열기 |
| Space | 베이스 정지·gate 해제; 그리퍼는 마지막 목표 유지 |
| q / Ctrl-C | 정지 요청 후 종료 |

회전과 리프트는 제외한다. 그리퍼 동작 동안 주행은 잠기며, 완료 후 키를 한 번 놓아야 다시 움직인다. 모터 명령 시작의 완벽한 동시성/물리 파지 성공은 보장하지 않는다.

서브 한 줄 준비: `~/damgc_robot_mission/scripts/run_follower_manual.sh`

**다운로드·설정·실행:** [리더 인계 문서](../../docs/MANUAL_STRAIGHT_TRANSPORT_HANDOFF.md).

- motion/drive/bridge/I2C/gripper는 기본 비활성화.
- 앞뒤 속도±0.03m/s, 공통 가감속0.03m/s², 의도적인 팔로워 지연 없음.
- 팔로워 guard의 `allow_reverse=true`를 확인해야 활성화한다.
- 입력·통신·명령 ACK0.25초, odom 수신0.35초 만료 시 정지. 통신 복구만으로 재출발하지 않는다.
- G/R은 양쪽 모터 준비 확인 후 session/action ID를 사용해 한 번씩 실행한다. 모터 ERROR/ACK3초 만료 시 베이스 정지, 마지막 그리퍼 목표 유지.
- 리더 그리퍼 집기값은 미보정 기본값-1이므로 `gripper_enabled=true` 사용 시 반드시 확정된 `close_raw`를 지정한다. 팔로워 기본 집기331, 열림1000.
- 기존 리더 mapping selector/bridge와 양쪽 Dynamixel 노드를 재사용한다. 이 패키지는 하드웨어 연결이나 Dynamixel 노드를 새로 만들지 않는다.

개회식 직선 운반(키보드 열기·파지·지속 직진): [사용법](../../docs/OPENING_CEREMONY_TRANSPORT.md).
