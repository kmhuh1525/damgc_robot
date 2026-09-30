#!/usr/bin/env bash
# Cooperative grasp -> lift -> 1 s transport mission launcher.
#   Follower Orin first:  bash scripts/run_cooperative_mission.sh follower
#   Leader Orin second:   bash scripts/run_cooperative_mission.sh leader
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROLE="${1:-}"

usage() {
  printf 'Usage: %s leader|follower\n' "${0##*/}"
  printf '\nEnvironment overrides:\n'
  printf '  ROS_DOMAIN_ID=42               Shared DDS domain (both Orins)\n'
  printf '  COOP_PEER_IP=192.168.0.7       Optional peer ping check\n'
  printf '  COOP_USE_STM32_BRIDGE=1        0 = no motor bridge (dry run)\n'
  printf '  COOP_I2C_DEVICE=/dev/i2c-7     STM32 I2C device\n'
  printf '  COOP_I2C_ADDRESS=66            STM32 7-bit address\n'
  printf '  COOP_I2C_WRITE_ENABLED=1       0 = bridge receive-only\n'
  printf '  MISSION_GRIPPER=1              0 = run without the Dynamixel gripper\n'
  printf '  MISSION_GRIPPER_PORT=/dev/ttyUSB0\n'
  printf '  MISSION_DIRECTION=forward      Leader-frame transport direction (forward|backward)\n'
  printf '  MISSION_SPEED=0.05             Transport cruise speed [m/s] (<= 0.10)\n'
  printf '  MISSION_DURATION=1.0           Transport window [s]\n'
  printf '  MISSION_DISCOVERY_TIMEOUT=30   Leader wait for the Follower [s]\n'
  printf '  MISSION_AUTOSTART=0            1 = start without pressing Enter\n'
}

if [[ "${ROLE}" != "leader" && "${ROLE}" != "follower" ]]; then
  usage >&2
  exit 2
fi

normalize_bool() {
  case "$1" in
    1|true|TRUE|yes|YES) printf 'true' ;;
    0|false|FALSE|no|NO) printf 'false' ;;
    *) printf 'Expected a boolean value, got: %s\n' "$1" >&2; return 2 ;;
  esac
}

source_with_nounset_disabled() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble setup was not found.\n' >&2
  exit 1
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  printf 'Workspace is not built. Run colcon build first.\n' >&2
  exit 1
fi
source_with_nounset_disabled /opt/ros/humble/setup.bash
source_with_nounset_disabled "${REPO_ROOT}/install/setup.bash"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros2_dds_env.sh"

use_bridge="$(normalize_bool "${COOP_USE_STM32_BRIDGE:-1}")"
i2c_write_enabled="$(normalize_bool "${COOP_I2C_WRITE_ENABLED:-1}")"
gripper_enabled="$(normalize_bool "${MISSION_GRIPPER:-1}")"
i2c_device="${COOP_I2C_DEVICE:-/dev/i2c-7}"
i2c_address="${COOP_I2C_ADDRESS:-66}"
gripper_port="${MISSION_GRIPPER_PORT:-/dev/ttyUSB0}"
direction="${MISSION_DIRECTION:-forward}"
speed="${MISSION_SPEED:-0.05}"
duration="${MISSION_DURATION:-1.0}"
discovery_timeout="${MISSION_DISCOVERY_TIMEOUT:-30}"

if [[ "${direction}" != "forward" && "${direction}" != "backward" ]]; then
  printf 'MISSION_DIRECTION must be forward or backward.\n' >&2
  exit 2
fi
if ! [[ "${discovery_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'MISSION_DISCOVERY_TIMEOUT must be a positive integer.\n' >&2
  exit 2
fi
if [[ -n "${COOP_PEER_IP:-}" ]]; then
  printf 'Checking peer %s...\n' "${COOP_PEER_IP}"
  if ! ping -c 1 -W 2 -- "${COOP_PEER_IP}" >/dev/null; then
    printf 'Peer ping failed: %s\n' "${COOP_PEER_IP}" >&2
    exit 1
  fi
fi

common_args=(
  "use_stm32_bridge:=${use_bridge}"
  "i2c_device:=${i2c_device}"
  "i2c_address:=${i2c_address}"
  "i2c_write_enabled:=${i2c_write_enabled}"
  "gripper_enabled:=${gripper_enabled}"
  "gripper_port:=${gripper_port}"
)

printf 'Role: %s, DDS domain: %s, STM32 bridge: %s, gripper: %s\n' \
  "${ROLE}" "${ROS_DOMAIN_ID}" "${use_bridge}" "${gripper_enabled}"

if [[ "${ROLE}" == "follower" ]]; then
  printf 'Follower waits for Leader mission commands (selector STOP, guard disabled).\n'
  exec ros2 launch cooperative_mission follower_mission.launch.py "${common_args[@]}"
fi

# ------------------------------------------------------------------ Leader
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/cooperative_mission_${RUN_ID}"
mkdir -p "${LOG_DIR}"
launch_pid=""

mission_state() {
  timeout 4s ros2 topic echo --once --qos-durability transient_local \
    --qos-reliability reliable /mission/state std_msgs/msg/String 2>/dev/null \
    | sed -n 's/^data: *//p' | tr -d "'\"" | head -n 1
}

mission_detail() {
  timeout 4s ros2 topic echo --once --qos-durability transient_local \
    --qos-reliability reliable /leader/mission/status std_msgs/msg/String 2>/dev/null \
    | sed -n 's/^data: *//p' | head -n 1
}

call_trigger() {
  timeout 5s ros2 service call "/mission/$1" std_srvs/srv/Trigger '{}' 2>&1 || true
}

cleanup() {
  trap - EXIT INT TERM
  printf '\nStopping Leader mission stack...\n'
  call_trigger abort >/dev/null
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 0.3
    done
    if kill -0 "${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    fi
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch cooperative_mission leader_mission.launch.py \
  "${common_args[@]}" \
  "transport_direction:=${direction}" \
  "transport_speed:=${speed}" \
  "transport_duration:=${duration}" \
  >"${LOG_DIR}/leader_mission.log" 2>&1 &
launch_pid=$!
printf 'Leader mission stack starting. Log: %s/leader_mission.log\n' "${LOG_DIR}"

deadline=$((SECONDS + discovery_timeout))
until ros2 service list 2>/dev/null | grep -Fxq '/mission/start'; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    printf 'Leader launch exited. Check %s/leader_mission.log\n' "${LOG_DIR}" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    printf 'Timed out waiting for /mission/start.\n' >&2
    exit 1
  fi
  sleep 1
done

printf 'Waiting up to %ss for the Follower mission executor...\n' "${discovery_timeout}"
if ! timeout "${discovery_timeout}s" ros2 topic echo --once /follower/mission/status \
    std_msgs/msg/String >/dev/null 2>&1; then
  printf 'Follower mission status not received. Start the Follower first and check DDS.\n' >&2
  exit 1
fi
sleep 3  # let cameras, TF and the gripper nodes settle

printf '\nScenario: Leader tag search/approach -> Leader grasp -> Follower opposite-face\n'
printf 'grasp -> simultaneous lift -> %s transport %s m/s for %s s.\n' "${direction}" "${speed}" "${duration}"
printf 'Keep the hardware E-stop within reach. Ctrl-C aborts (motion stops, grippers hold).\n'
if [[ "${MISSION_AUTOSTART:-0}" != "1" ]]; then
  read -r -p 'Press Enter to START the mission... ' _
fi
result="$(call_trigger start)"
printf '%s\n' "${result}"
if [[ "${result}" != *"success=True"* && "${result}" != *"success: true"* ]]; then
  printf 'Mission start was rejected. Fix the reason above and re-run.\n' >&2
  exit 1
fi

last=""
while true; do
  state="$(mission_state)"
  if [[ -n "${state}" && "${state}" != "${last}" ]]; then
    printf '[%s] mission state: %s\n' "$(date +%H:%M:%S)" "${state}"
    last="${state}"
  fi
  if [[ "${state}" == "DONE" || "${state}" == "FAULT" ]]; then
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    printf 'Leader launch exited unexpectedly. Check the log.\n' >&2
    exit 1
  fi
  sleep 0.5
done

printf 'Detail: %s\n' "$(mission_detail)"
if [[ "${state}" == "FAULT" ]]; then
  printf '\nMission FAULT: both robots are stopped and grippers keep holding.\n'
  printf 'To lower and open both grippers together:  ros2 service call /mission/release std_srvs/srv/Trigger\n'
fi
printf '\nNOTE: stopping this script turns off Dynamixel torque (the object would drop).\n'
read -r -p 'Press Enter to LOWER and RELEASE the object together... ' _
call_trigger release
release_deadline=$((SECONDS + 30))
while true; do
  state="$(mission_state)"
  [[ "${state}" == "IDLE" ]] && break
  if [[ "${state}" == "FAULT" ]] || (( SECONDS >= release_deadline )); then
    printf 'Release did not complete (state=%s): %s\n' "${state}" "$(mission_detail)" >&2
    break
  fi
  sleep 0.5
done
printf 'Mission state: %s. Press Ctrl-C to exit.\n' "${state}"
wait "${launch_pid}"
