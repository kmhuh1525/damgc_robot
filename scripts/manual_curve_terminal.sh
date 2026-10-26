#!/usr/bin/env bash
# Leader existing drive + new measured-trail curve controller. No new motor bus.
set -Eeuo pipefail
curve_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${curve_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
curve_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$curve_pid" ]]; then kill -INT "$curve_pid" 2>/dev/null || true; wait "$curve_pid" || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Latest leader main already calibrates its gripper at477 in manual_straight_terminal.sh.
ros2 run cooperative_manual manual_curve --ros-args -r __ns:=/leader \
  -p role:=leader -p motion_enabled:=true -p odom_topic:="${LEADER_ODOM_TOPIC:-/leader/odom/raw}" \
  -p gripper_enabled:=true -p close_raw:="${LEADER_CLOSE_RAW:-477}" -p gripper_settle:=9.0 \
  > /tmp/leader_manual_curve.log 2>&1 &
curve_pid=$!
printf '곡선 운반 실험 모드: 서브 curve 준비 후 e, ↑/w 당기기, ←/a·→/d 완만한 곡선. Space 정지.\n'
ros2 run cooperative_manual manual_curve_keys
