#!/usr/bin/env bash
# Scenario-controlled paired grip and measured-path curve transport.
set -Eeuo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${demo_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
demo_nodes="$(ros2 node list)"
if [[ "$demo_nodes" == *'/leader/manual_curve'* || "$demo_nodes" == *'/leader/manual_straight'* || "$demo_nodes" == *'/manual_curve_keys'* || "$demo_nodes" == *'/follower/manual_straight'* ]]; then
  printf '기존 리더 협동/키보드 및 서브 직진 태스크를 먼저 종료하세요. 서브 curve peer는 재사용합니다.\n' >&2
  exit 1
fi
exec ros2 run cooperative_manual demo_transport --ros-args -r __ns:=/leader \
  -p role:=leader -p motion_enabled:=true -p odom_topic:="${LEADER_ODOM_TOPIC:-/leader/odom/raw}" \
  -p gripper_enabled:=true -p close_raw:="${LEADER_CLOSE_RAW:-477}" -p gripper_settle:=9.0 \
  -p demo_command_topic:="${DEMO_COMMAND_TOPIC:-/leader/demo/cmd_vel}"
