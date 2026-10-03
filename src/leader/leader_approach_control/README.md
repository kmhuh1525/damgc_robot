# leader_approach_control

Leader의 hybrid center/tag-normal control target, internal mode와 alignment state를 사용해
`/leader/approach/cmd_vel_raw`을 계산하는 differential-drive controller다. 같은
패키지의 독립 `velocity_guard`가 `/leader/approach/cmd_vel_safe`의 clamp, reverse
차단, slew limit와 watchdog을 담당한다. Leader command selector는 이 입력과
TELEOP/NAV2 중 선택된 한 source만 `/leader/cmd_vel`에 발행한다.

## Interface

- Input target: `/leader/alignment/control_target_pose`
  (`geometry_msgs/msg/PoseStamped`)
- Input state: `/leader/base_alignment/state` (`std_msgs/msg/String`)
- Input mode: `/leader/alignment/control_mode` (`std_msgs/msg/String`)
- Safety gates: `/leader/supply/detected`, `/leader/supply/tag_id`
- Raw output: `/leader/approach/cmd_vel_raw` (`geometry_msgs/msg/Twist`)
- Controller enable: `/leader/approach/enable` (`std_srvs/srv/SetBool`)
- Guard output: `/leader/approach/cmd_vel_safe`
- Guard enable: `/leader/velocity_guard/enable` (`std_srvs/srv/SetBool`)

Controller는 target pose 뒤에 도착한 mode와 state를 하나의 generation으로 결합한다.
Stamp/receipt timeout, ID 변경, invalid pose, unknown mode/state 또는 disabled gate에서는
zero를 발행한다. Target pose의 orientation은 raw AprilTag yaw가 아니라 perception이
tag normal을 `base_link` XY로 projection해 만든 명시적 robot target yaw다.

## State별 command

| State | `linear.x` | `angular.z` |
|---|---:|---:|
| `TURN_LEFT/RIGHT`, `COARSE_TRACK` | 0 | Tag center bearing, 최대 `0.20 rad/s` |
| `APPROACH`, `COARSE_TRACK` | positive, 최대 `0.05 m/s` | Tag center bearing |
| `APPROACH`, `NEAR_ALIGN` | warning 영역에서 감소 | center + bounded normal, 최대 `0.10 rad/s` |
| `TURN_LEFT/RIGHT`, `RECENTER` | 0 | Tag center bearing, 최대 `0.10 rad/s` |
| `FINE_ALIGN_LEFT/RIGHT` | 0 | final target yaw, 최대 `0.08 rad/s` |
| `FINAL_APPROACH` | positive, 최대 `0.02 m/s` | yaw+lateral correction, 최대 `0.08 rad/s` |
| `TAG_LOST`, `TOO_CLOSE`, `STABILIZING`, `ALIGNED` | 0 | 0 |

Controller는 후진을 생성하지 않고 `linear.x`, `angular.z`만 채운다. 거리와 tolerance는
`rescue_robot_apriltag`가 소유하며 controller에 중복 `target_forward`가 없다.

## Build and test

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_apriltag leader_approach_control rescue_robot_bringup
source install/local_setup.bash
colcon test --packages-select rescue_robot_apriltag leader_approach_control
colcon test-result --verbose
```

실차 단계별 검증은
[`LEADER_HYBRID_TAG_ALIGNMENT_VALIDATION_GUIDE.md`](../rescue_robot_apriltag/docs/LEADER_HYBRID_TAG_ALIGNMENT_VALIDATION_GUIDE.md)를 따른다.
