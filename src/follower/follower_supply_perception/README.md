# follower_supply_perception

ROS 2 Humble 팔로워 AprilTag 인식·정렬·접근 패키지. 2026-10-27 실기에서 시작 서비스 한 번으로 인식부터 자동 집기까지 확인했습니다.

**최신 실행·파라미터·리더 인계 문서:** [팔로워 자동 집기 실기 인계](../../../docs/FOLLOWER_GRIPPER_DEMO_2026-10-27.md)

## 그리퍼 단독 시연

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot_mission/install/setup.bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py
```

별도 터미널에서 같은 환경을 source한 뒤:

```bash
ros2 service call /follower/grasp/start std_srvs/srv/Trigger '{}'
ros2 topic echo /follower/sequence/status
# 취소: 베이스 정지, 그리퍼 마지막 목표 유지
ros2 service call /follower/grasp/cancel std_srvs/srv/Trigger '{}'
```

그리퍼 열림1000 → 열림9초 대기 → 기존 자동 접근·정렬 → ALIGNED와 최신 카메라 거리0.20±0.02m 확인 → 가드 OFF·selector STOP 응답 → 거리 재확인 → 그리퍼 집기331 → 닫힘9초 대기 → DONE. 속도40, 리프트 비활성화입니다. DONE 이후 재검출만으로 열리지 않고 다음 시작 명령을 기다립니다. 모터 ERROR/ACK 미수신 또는 태스크 시간 초과 시 FAULT입니다. DONE은 상자 접촉 센서 판정이 아닌 명령과 대기 완료입니다.

| 설정 | 현재 기본값 |
|---|---:|
| `gripper_open_raw` / `gripper_close_raw` | 1000 / 331 |
| `rx28_speed` | 40 |
| `gripper_close_wait` / task `open_wait` | 9.0s / 9.0s |
| `lift_enabled` | false |
| `target_distance` / `base_target_forward` | 0.20m / 0.242m |
| `align_tag_normal` | false: 태그 중심으로 직접 접근 |
| `observation_hold_time` / `tag_receipt_timeout` | 0.25s / 0.60s |
| `blind_final_approach_enabled` | false |

RX-28 기계적 닫힘 끝점278과 실제 상자 집기331을 구분합니다. RX-64 참고 범위는45–286이지만 현재 시연에서는 움직이지 않습니다. 속도는 연결 시 적용되므로 실행 중 파라미터만 바꾸는 것으로 모터 레지스터가 바뀌지는 않습니다.

입력은 `usb_cam → camera_pair_node → apriltag_ros → apriltag_approach`입니다. camera_pair는 원본 시각을 유지한 mono8 왜곡 보정 영상과 CameraInfo를 동일 header로 발행합니다. 영상 확인 토픽은 `/follower/tag_camera/image_rect`입니다. 임시 TF/CameraInfo 보조 발행기는 함께 실행하지 않습니다.

접근 controller와 velocity guard는 비활성화, selector는 STOP으로 시작합니다. 시연에서는 시작 서비스가 이들을 순서대로 제어하므로 별도로 guard를 enable하지 않습니다. 수동 raw 명령이나 협동 미션 executor와 명령 주체를 겹치지 않습니다.

## 인식만 실행

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
# 기존 카메라·TF가 있을 때 상태 판단만 실행
ros2 launch follower_supply_perception approach_only.launch.py
```

## 설계·과거 검증 기록

아래 문서의 과거 기본값과 NOT VERIFIED 표기는 당시 기록입니다. 최신 실기 확인 범위와 제한은 상단 인계 문서를 따릅니다.

- [접근 상태 설계](docs/TASK_SPEC_APRILTAG_APPROACH.md)
- [노드 가이드](docs/APRILTAG_APPROACH_NODE_GUIDE.md)
- [base-link 속도 파이프라인](docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)
- [Hybrid alignment 이식 기록](docs/FOLLOWER_HYBRID_ALIGNMENT_MIGRATION.md)
- [과거 launch 검증](docs/LAUNCH_VALIDATION_APRILTAG.md)
- [실카메라 수동 상태 시험](docs/MANUAL_STATE_TEST.md)
