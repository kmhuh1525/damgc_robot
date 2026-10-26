#!/usr/bin/env bash
# Reuse the existing drive/motor stack, or cold-start it once.
set -Eeuo pipefail
opening_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${opening_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
exec 9>"/tmp/cooperative_manual_follower_${UID}_${ROS_DOMAIN_ID}.lock"
flock -n 9 || { printf '서브 준비 명령이 이미 실행 중입니다.\n' >&2; exit 1; }
opening_status=0
ros2 run cooperative_manual opening_ceremony_probe || opening_status=$?
case "$opening_status" in
  0) exec ros2 run cooperative_manual follower_prepare ;;
  10) flock -u 9; exec "${opening_root}/scripts/run_follower_manual.sh" "$@" ;;
  12) ;;
  *) exit "$opening_status" ;;
esac
opening_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$opening_pid" ]]; then kill -INT "$opening_pid" 2>/dev/null || true; wait "$opening_pid" || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch cooperative_manual manual_straight.launch.py role:=follower motion_enabled:=true gripper_enabled:=true close_raw:=331 gripper_settle:=9.0 &
opening_pid=$!
ros2 run cooperative_manual follower_prepare
printf '개회식 서브 준비 완료. 이 터미널을 유지하세요.\n'
wait "$opening_pid"
