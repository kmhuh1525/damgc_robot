# AprilTag 파이프라인 launch 검증 기록

## 구성

`follower_apriltag.launch.py`는 다음 네 노드를 재현 가능한 하나의 구성으로 실행한다.

- `/follower/camera/usb_cam`: `/dev/video0`, 640x480, 30 fps, mmap, native `yuyv`
- `/follower/camera/rectify_node`: image_raw와 camera_info를 image_rect로 보정
- `/follower/apriltag/apriltag`: 36h11, tag size `0.050 m`
- `/follower/apriltag_approach`: TF 기반 상태 판단

RViz, `cmd_vel`, STM32와 그리퍼 제어는 포함하지 않는다.

## 보존한 설정

기존 원본 `/home/kde/apriltag_config/apriltag.yaml`은 수정하거나 삭제하지 않았다.
실행에는 패키지 내부 복사본 `config/apriltag.yaml`을 사용한다. 당시 설치 복사본까지 SHA-256
`cde624e98c3e6beb60ea1c0ca1f405e4728b3cb087a025f20ceb47af0f39cb45`로
동일함을 확인했다. 카메라 보정 파일
카메라 보정값은 이후 패키지 내부 `config/follower_usb_camera.yaml`로 편입했으며,
기본 launch는 작업공간 밖의 보정 파일을 참조하지 않는다.

## 자동 검증 결과

- 두 Python launch 파일 compile 성공
- `colcon build --packages-select follower_supply_perception` 성공
- 전체 launch `--show-args` 성공
  - `video_device`
  - `camera_info_url`
  - `apriltag_config`
  - `approach_config`
- approach-only `--show-args` 성공
  - `approach_config`
- install 공간에서 두 launch와 두 YAML 발견
- approach-only launch를 timeout으로 실제 실행
  - 상태 노드 시작 확인
  - `TAG_LOST` 초기 상태 확인
  - SIGINT 후 child process clean 종료 확인

샌드박스 검증에서는 ROS 기본 로그 경로 대신 허용된
`ROS_LOG_DIR=/home/kde/ros2_ws/log`를 사용했다. 실제 운영 터미널에서는 기본 ROS
로그 경로를 그대로 사용할 수 있다.

## 전체 launch 실제 실행 결과

첫 확인에서는 다음 수동 노드가 실행 중이어서 자동 종료하거나 전체 launch를 시작하지
않았다.

- `/follower/camera/usb_cam`
- `/RectifyNode`
- `/follower/apriltag/apriltag`

사용자가 수동 노드를 종료한 뒤 중복이 없음을 재확인하고 전체 launch를 timeout 제한
세션에서 실제 실행했다. 다음 네 노드가 함께 시작됐다.

- `/follower/camera/usb_cam`
- `/follower/camera/rectify_node`
- `/follower/apriltag/apriltag`
- `/follower/apriltag_approach`

실행 중 `/follower/camera/image_raw`, `camera_info`, `image_rect`,
`/follower/apriltag/detections`, 8개 상태 판단 출력과 `/tf`를 확인했다.
`image_rect`는 관찰 구간에서 약 `11.8 Hz`, AprilTag `size`는 `0.05`였다. 카메라는
`/dev/video0`, 640x480, 30 fps, mmap, native `yuyv`로 시작했다. 이 장치는 같은
해상도의 MJPEG를 120.101 fps로만 제공하므로 30 fps pipeline에서는 사용하지 않는다.

카메라가 지원하지 않는 일부 선택 제어에 `unknown control` 경고가 있었지만 영상과
필수 토픽은 정상 생성됐다. 종료 시 timeout과 명시적 interrupt가 거의 동시에 들어가
rclpy Future 소멸 경고가 한 번 출력됐으나 네 child 모두 `process has finished cleanly`로
종료됐다. 이후 프로세스 검색과 ROS node list에서 launch child가 남지 않았음을
확인했다.

## 설치 경로

- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/launch/follower_apriltag.launch.py`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/launch/approach_only.launch.py`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/apriltag.yaml`
- `/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml`

## 운영 명령

전체 파이프라인:

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

장치 또는 설정 경로 override:

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py \
  video_device:=/dev/video0 \
  camera_info_url:=file:///home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/follower_usb_camera.yaml \
  apriltag_config:=/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/apriltag.yaml \
  approach_config:=/home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml
```

기존 카메라·AprilTag 파이프라인에 상태 노드만 추가:

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

영상 확인은 launch와 별도 터미널에서 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```

## 현재 drive launch와 grace parameter 확인

Hybrid command pipeline은 별도 `follower_apriltag_drive.launch.py`가 구성한다. 이 launch의
`--show-args`에서 `use_stm32_bridge`와 `i2c_write_enabled`가 실제 true/false argument로
지원됨을 확인했다. 테스트 명령에 이 인자를 사용할 때는 항상 먼저 현재 launch의
`--show-args` 결과를 확인하며, 이를 위해 bridge 또는 launch safety semantics를 바꾸지
않는다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py --show-args
ros2 param get /follower/apriltag_approach stabilizing_tag_loss_grace_sec
ros2 param get /follower/apriltag_approach final_approach_tag_loss_grace_sec
ros2 param get /follower/apriltag_approach blind_final_approach_enabled
```

현재 기대값은 grace 두 개 모두 `0.30`, blind final은 `False`다. Controller
enable/disable은 `/follower/approach/enabled`를 통해 perception의 approach-session reset을
발생시키며, launch 시작 또는 session 전환 후 과거 ALIGNED latch를 재사용하지 않는다.

## Follower Dynamixel integration 안전 계약

`follower_apriltag_drive.launch.py`에는 기존부터 Dynamixel controller와 gripper sequence가
조건부 통합되어 있다. 현재 감사에서는 이 연결을 유지하고 Follower의 기본 actuator
정책만 다음과 같이 고정했다.

- `gripper_enabled=true`, `robot=follower`, OPEN `950`, CLOSE `350`
- Historical validation used `lift_raw=300`, `rx64_speed=50` under the former `260..670` profile.
- startup pose/torque disabled: Tag 전 RX-64/RX-28 position/torque write 없음
- Tag-loss idle disabled: detection flicker에 따른 reposition 없음
- OPEN `[-1,950,-1,1]`, CLOSE `[-1,350,-1,1]`
- sequence status `/follower/sequence/status`
- wheel velocity guard startup disabled 유지

RX-64 Moving Speed는 Protocol 1.0 주소 `32`에 2-byte raw 값 `50`을 연결 시 한 번 쓴다.
당시에는 CLOSE 후 `close_wait=3.0 s`가 지나면 targeted Goal Position command
`[300,-1,1,-1]`을 정확히 한 번 발행하여 `LIFTING → DONE`으로 진행했다. 이 기록의
RX-64 `500 → 300` hardware 동작과 speed `50` 검증은 이전 `260..670` profile 기준이다.
현재 follower profile은 raw `20..270`이며 `20=raised`, `270=lowered`로 설정되었다.
RX-28 Moving Speed register에는 write하지 않는다.

`lift_enabled:=false`는 OPEN/CLOSE까지만 수행하고 RX-64 Goal Position을 생략한다.
`gripper_enabled:=false`는 speed 설정을 포함한 두 gripper child를 모두 제외한다. 실제
hardware 절차는 README를 따르며 자동 검증에서는 actuator 또는 wheel command를 발행하지
않는다.

2026-09-07 software-only 검증 결과:

- 변경 package `rescue_robot_tools`, `follower_supply_perception` symlink build 성공
- shared Dynamixel/sequence unit test 18개 성공
- Follower package test 143개 성공
- Leader integrated launch regression test 성공
- `ros2 launch ... --show-args`로 top-level/child argument와 package resolution 확인
- master gate의 두 child condition이 `false`에서 모두 제외되고 `true`에서 모두
  포함되는 것을 launch context로 확인
- focused shared/Follower/Leader regression: 33 tests passed
- `colcon test-result --verbose`: 444 tests, 0 errors, 0 failures, 0 skipped
- 실제 U2D2, Dynamixel, wheel 또는 integrated robot process는 실행하지 않음

2026-09-08 RX-64 speed/default lift software-only 검증 결과:

- 변경 package `rescue_robot_tools`, `rescue_robot_bringup`,
  `follower_supply_perception` symlink build 성공
- focused shared/Leader/Follower tests: 47 passed
- Leader `--show-args` historical result: `lift_enabled=true`, `lift_raw=300`,
  `rx64_speed=50` 확인
- `colcon test-result --verbose`: 549 tests, 0 errors, 0 failures, 0 skipped
- mock SDK로 RX-64 ID/주소 32/2-byte/raw 50 및 통신 오류 확인
- RX-28 Moving Speed write 없음과 lift/master-gate disable regression 확인
- 실제 U2D2, Dynamixel, wheel 또는 integrated robot process는 실행하지 않음
