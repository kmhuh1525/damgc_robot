#!/usr/bin/env bash
# One command: launch fresh stack or reuse a complete existing manual stack.
set -Eeuo pipefail
manual_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${manual_root}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
if [[ "${1:-}" == --check-only ]]; then
  exec ros2 run cooperative_manual follower_prepare --mode curve --check-only
fi
if ros2 run cooperative_manual follower_prepare --mode curve --probe; then
  exec ros2 run cooperative_manual follower_prepare --mode curve
else
  manual_probe_status=$?
  if [[ "$manual_probe_status" != 10 ]]; then exit "$manual_probe_status"; fi
fi
# Prevent simultaneous cold starts; no lock is needed for the reuse path.
exec 9>"/tmp/cooperative_manual_follower_${UID}_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then printf '다른 터미널에서 서브 구성을 준비 중입니다.\n' >&2; exit 1; fi
manual_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$manual_pid" ]] && kill -0 "$manual_pid" 2>/dev/null; then
    kill -INT "$manual_pid" 2>/dev/null || true
    wait "$manual_pid" || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch cooperative_manual follower_curve_ready.launch.py "$@" &
manual_pid=$!
ros2 run cooperative_manual follower_prepare --mode curve
printf '\n서브 실행 중입니다. 이 터미널은 유지하세요. Ctrl-C는 베이스를 정지하고 종료합니다.\n'
wait "$manual_pid"
