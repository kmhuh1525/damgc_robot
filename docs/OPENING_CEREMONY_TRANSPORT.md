# 개회식 직선 협동 운반 사용법

정해진 위치에 두 로봇과 상자를 배치한 뒤, 키보드 하나로 **양쪽 그리퍼 열기 → 양쪽 파지 → 직선 이동 → 정지 → 내려놓기**를 수행한다. 위치 인식·AprilTag·Nav2 없이 기존 `manual_straight` 협동 제어와 양쪽 파지 프로토콜을 재사용한다. 리프트와 회전은 사용하지 않는다.

## 배치와 방향

리더가 이동 방향의 앞에 있고, 뒤쪽의 서브와 서로 마주 보는 배치다. ↑는 **리더 후진·서브 전진**으로 상자를 끌고 이동한다. ↓는 **리더 전진·서브 후진**으로 직선 복귀한다. 다른 배치라면 실제 진행 방향부터 확인한다. 별도 경로 추종이나 방위 보정 없이 양쪽에 반대 부호의 같은 선속도를 전달하므로, 바퀴 미끄러짐이나 시작 자세 차이는 자동 보정하지 않는다.

- 주행 속도: 기본0.03 m/s, 키보드로0.01~0.03 m/s 조절 (약 1m에 33초, 가감속 시간 제외).
- 공통 선가감속 제한: 0.03 m/s². 전후진 전환 시 0을 거친다.
- 리더 파지477, 서브 파지331, 양쪽 열림1000. 동작 대기9초 및 양쪽 완료 확인.
- 서브 그리퍼 속도40, 리프트 제외. 리더 모터 노드는 기존 설정을 사용한다.
- 곡선 모드에서 사용하는 실측 속도 기반 추종·위치 오차 보정은 이 직진 모드에서 사용하지 않는다. 모터 자체의 저속 떨림이 해결됐다는 뜻은 아니다.

## 코드 업데이트와 빌드

리더 저장소: `https://github.com/rbgusrns/damgc_robot`, 작업 브랜치 `codex/manual-curve-20261027`.
서브 저장소: `https://github.com/kmhuh1525/damgc_robot`, 작업 브랜치 `codex/cooperative-mission-20260930`.
이름에 curve가 있어도 리더 작업 브랜치에는 이 개회식 모드가 포함되어 있다. main에는 자동 병합하지 않는다. 리더 작업자는 아래 파일/커밋을 자신의 작업 브랜치에 반영한 뒤 빌드한다.

리더 반영 파일: `src/cooperative_manual/cooperative_manual/keyboard.py`, `src/cooperative_manual/setup.py`, `scripts/opening_ceremony_terminal.sh`. 패키지 자체가 없는 구버전이면 기존 `cooperative_manual` 패키지도 먼저 반영해야 한다.

각 로봇의 워크스페이스 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cooperative_manual --symlink-install
source install/setup.bash
```

## 1. 기존 곡선/수동 태스크 종료

양쪽 베이스를 정지한 뒤 기존 곡선·수동 키보드/협동 노드 실행 터미널에서 Ctrl-C로 종료한다. 모터·브리지·selector·guard 노드는 재사용할 수 있다. 다른 협동 태스크와 동시에 실행하지 않는다. 새 스크립트는 곡선 노드나 중복 구동 노드를 발견하면 시작을 거부하며 임의로 종료하지 않는다.

현재 서브에 곡선 노드만 따로 실행된 구성에서는 다음으로 해당 노드만 종료할 수 있다. 그리퍼 목표와 모터 노드는 유지된다.

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot_mission/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 service call /follower/manual_curve/enable std_srvs/srv/SetBool '{data: false}'
pkill -INT -f '/lib/cooperative_manual/manual_curve --ros-args -r __node:=manual_curve -r __ns:=/follower'
```

## 2. 서브 준비

서브 터미널:

```bash
~/damgc_robot_mission/scripts/run_follower_opening.sh
```

서브 구동·모터 노드가 모두 있으면 재사용하고 직진 협동 노드만 추가한다. 아무것도 없으면 전체 서브 구성을 시작한다. 이미 완전한 직진 구성이 있으면 재사용한다. 누락/중복/다른 태스크가 있으면 오류를 출력한다. 완료 화면의 `WAIT_PEER`는 정상이며 출력은0이다. 실행을 유지하는 터미널은 닫지 않는다.

## 3. 리더 키보드 실행

리더에는 기존 구동·selector·Dynamixel 및 `/leader/odom/raw`가 실행 중이어야 한다. 리더 스크립트는 하드웨어 노드를 중복으로 켜지 않는다.

```bash
~/damgc_robot/scripts/opening_ceremony_terminal.sh
```

워크스페이스 폴더명이 다르면 해당 폴더의 같은 스크립트를 실행한다. 환경은 기본 ROS domain0, FastDDS UDPv4다. 필요하면 `ROS_DOMAIN_ID`를 양쪽 동일하게 지정한다.

## 4. 실제 퍼포먼스 순서

| 순서/키 | 동작·확인 |
|---|---|
| `E` | 서브 준비 후 리더 활성화. `DRIVE ACTIVE` 확인. |
| `R` | 정지 확인 후 양쪽 그리퍼 열기. `GRIP DONE`까지 기다린다. |
| 상자 배치 | 열린 그리퍼 사이에 상자를 맞춘다. 로봇 시작 방향도 맞춘다. |
| `G` | 양쪽 파지. 약9초와 양쪽 완료 확인 후 `GRIP DONE`. |
| `↑` 또는 `W` 한 번 | 계속 직진. 리더 후진, 서브 전진. **키를 놓아도 계속 간다.** |
| `+` / `=` · `-` | 선택 속도를 0.005 m/s씩 증가·감소. 주행 중에도 방향을 유지하며 기존 가감속 제한으로 반영. 현재 선택 속도를 화면에 표시한다. |
| `1` / `2` / `3` | 각각0.01 /0.02 /0.03 m/s 즉시 선택. 정지 중에는 속도만 바뀌고 출발하지 않는다. |
| `X` | 양쪽 감속 정지. 도착하면 먼저 누른다. |
| `R` | 완전히 정지한 뒤 양쪽 열기. `GRIP DONE` 확인 후 상자를 뺀다. |
| `Q` | 베이스 정지 요청 후 개회식 키보드/리더 협동 노드 종료. |

↓/S 한 번은 반대 방향 직선 복귀다. ←/→/A/D는 회전하지 않고 감속 정지한다. 파지·열기 도중에는 이동이 잠기며, 완료 후 X를 눌렀다가 ↑를 누르면 다시 움직인다. 주행 키를 파지 완료 전에 미리 누르지 않는다.

**Space는 즉시 양쪽 베이스 정지·협동 비활성화**다. X는 활성 상태를 유지한 일반 정지라 ↑로 다시 출발할 수 있다. 그리퍼는 정지/종료 시 마지막 목표를 유지한다.

속도 상한은 기존 양쪽 직진 제어 설정과 같은0.03 m/s다. +를 더 눌러도 상한을 넘지 않는다. 표시값은 목표 속도이며 실제 바퀴 속도는 가감속·부하에 따라 다를 수 있다. 그리퍼 속도는 이 키로 바뀌지 않는다.

## 정지 후 다시 준비하기

- X로 멈췄다면 ↑ 또는 ↓만 누르면 된다.
- Space/통신 오류로 멈췄다면 서브에서 `run_follower_opening.sh`를 다시 실행하여 준비한 뒤 리더 키보드에서 E를 누른다. 이후 G/R 또는 이동 키를 누른다.
- `GRIP REJECTED`이면 ACTIVE 상태와 양쪽 모터 준비, 베이스 정지 여부를 확인한다.
- `GRIP ERROR`/통신·odom 오류는 정지를 유지한다. 원인 해결 후 양쪽 재준비한다. 연결 복구만으로 자동 출발하지 않는다.
- 입력 노드가 죽거나 통신이 끊어지면 watchdog이 정지한다. 키를 놓는 것은 이 모드에서 정지 조건이 아니다.

## 확인 범위와 로그

격리 ROS domain176의 가상 터미널 테스트로 ↑ 한 번 후 지속 명령, X 정지, ↓ 반대 방향, G/R 요청 시 주행0, Space 비활성화 및 종료를 확인한다. 기존 직진 협동/양쪽 파지 회귀 테스트도 실행한다. 개회식 모드의 실제 상자 운반은 아직 검증 전이다.

리더 로그: `/tmp/leader_opening_ceremony.log`. 일반 직진 서비스는 `/leader/manual_straight/enable`, `/follower/manual_straight/enable`을 재사용한다. 메시지 경로도 `/cooperation/manual/*`이며 별도 곡선 태스크와 섞어 쓰지 않는다.
