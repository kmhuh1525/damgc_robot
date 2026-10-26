#!/usr/bin/env bash
# Leader one-key straight ceremony task; existing hardware processes are reused.
set -Eeuo pipefail
opening_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${opening_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Do not launch a second peer over another transport task.
opening_nodes="$(ros2 node list)"
if [[ "$opening_nodes" == *'/leader/manual_curve'* || "$opening_nodes" == *'/leader/manual_straight'* || "$opening_nodes" == *'/follower/manual_curve'* ]]; then
  printf '기존 직진/곡선 태스크를 먼저 종료하세요. 구동·모터 노드는 그대로 재사용합니다.\n' >&2
  exit 1
fi
opening_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$opening_pid" ]]; then kill -INT "$opening_pid" 2>/dev/null || true; wait "$opening_pid" || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 run cooperative_manual manual_straight --ros-args -r __ns:=/leader \
  -p role:=leader -p motion_enabled:=true -p odom_topic:="${LEADER_ODOM_TOPIC:-/leader/odom/raw}" \
  -p gripper_enabled:=true -p close_raw:="${LEADER_CLOSE_RAW:-477}" -p gripper_settle:=9.0 \
  > /tmp/leader_opening_ceremony.log 2>&1 &
opening_pid=$!
ros2 run cooperative_manual opening_ceremony_keys
