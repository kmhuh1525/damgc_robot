#!/usr/bin/env bash

set -Eeuo pipefail

REPO_ROOT="/home/maze/damgc_robot"
CONTAINER_NAME="${ISAAC_CONTAINER_NAME:-isaac_ros_dev-$(uname -m)-container}"
MAPPING_IMAGE="${ISAAC_MAPPING_IMAGE:-damgc-vslam-mapping:humble}"
MAPPING_DOCKERFILE="${REPO_ROOT}/docker/vslam_mapping.Dockerfile"
RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}/damgc-vslam-mapping-${UID}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/vslam_mapping_${RUN_ID}"
LAUNCHER_PID_FILE="${RUNTIME_ROOT}/launcher.pid"
CONTAINER_VSLAM_PID_FILE="/tmp/damgc_vslam_mapping_vslam.pid"
CONTAINER_RVIZ_PID_FILE="/tmp/damgc_vslam_mapping_rviz.pid"
CONTAINER_BAG_PID_FILE="/tmp/damgc_vslam_mapping_bag.pid"
BAG_DIR="${REPO_ROOT}/data/vslam_mapping_${RUN_ID}"
CONTAINER_BAG_DIR="/workspaces/isaac_ros-dev/data/vslam_mapping_${RUN_ID}"

HOST_PIDS=()
CONTAINER_STARTED_BY_US=0
CONTAINER_USER=""
CLEANING_UP=0
BAG_STARTED=0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
VSLAM_HEADLESS="${VSLAM_HEADLESS:-0}"
VSLAM_ONLY="${VSLAM_ONLY:-0}"
STM32_I2C_DEVICE="${STM32_I2C_DEVICE:-/dev/i2c-7}"
STM32_I2C_ADDRESS="${STM32_I2C_ADDRESS:-66}"
STM32_I2C_POLL_HZ="${STM32_I2C_POLL_HZ:-500.0}"
STM32_I2C_WRITE_ENABLED="${STM32_I2C_WRITE_ENABLED:-1}"

if [[ "${VSLAM_HEADLESS}" != "0" && "${VSLAM_HEADLESS}" != "1" ]] || \
  [[ "${VSLAM_ONLY}" != "0" && "${VSLAM_ONLY}" != "1" ]]; then
  printf 'VSLAM_HEADLESS and VSLAM_ONLY must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${VSLAM_ONLY}" == "1" && "${VSLAM_HEADLESS}" != "1" ]]; then
  printf 'VSLAM_ONLY=1 also requires VSLAM_HEADLESS=1.\n' >&2
  exit 1
fi
if [[ "${STM32_I2C_WRITE_ENABLED}" != "0" && \
      "${STM32_I2C_WRITE_ENABLED}" != "1" ]]; then
  printf 'STM32_I2C_WRITE_ENABLED must be 0 or 1.\n' >&2
  exit 1
fi
STM32_I2C_WRITE_ARG="false"
if [[ "${STM32_I2C_WRITE_ENABLED}" == "1" ]]; then
  STM32_I2C_WRITE_ARG="true"
fi

mkdir -p "${RUNTIME_ROOT}" "${LOG_DIR}"

is_container_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" == "true" ]]
}

source_with_nounset_disabled() {
  local setup_file="$1"

  # ROS 2 Humble setup scripts probe optional variables without always using
  # ${name:-}. Temporarily disable nounset while sourcing them.
  set +u
  source "${setup_file}"
  set -u
}

stop_container_process() {
  local pid_file="$1"
  local signal="$2"
  local container_pid

  container_pid="$(docker exec "${CONTAINER_NAME}" sh -c "cat '${pid_file}' 2>/dev/null" 2>/dev/null || true)"
  if [[ "${container_pid}" =~ ^[0-9]+$ ]]; then
    docker exec "${CONTAINER_NAME}" kill "-${signal}" "${container_pid}" >/dev/null 2>&1 || true
  fi
}

container_process_running() {
  local pid_file="$1"
  local container_pid

  container_pid="$(docker exec "${CONTAINER_NAME}" sh -c "cat '${pid_file}' 2>/dev/null" 2>/dev/null || true)"
  [[ "${container_pid}" =~ ^[0-9]+$ ]] && \
    docker exec "${CONTAINER_NAME}" kill -0 "${container_pid}" >/dev/null 2>&1
}

finalize_bag() {
  local deadline

  if (( ! BAG_STARTED )) || ! is_container_running; then
    return
  fi

  printf 'Finalizing rosbag...\n'
  stop_container_process "${CONTAINER_BAG_PID_FILE}" INT
  deadline=$((SECONDS + 20))
  while container_process_running "${CONTAINER_BAG_PID_FILE}"; do
    if (( SECONDS >= deadline )); then
      printf 'Rosbag did not stop within 20 seconds; sending TERM.\n' >&2
      stop_container_process "${CONTAINER_BAG_PID_FILE}" TERM
      deadline=$((SECONDS + 5))
      while container_process_running "${CONTAINER_BAG_PID_FILE}" && (( SECONDS < deadline )); do
        sleep 1
      done
      break
    fi
    sleep 1
  done

  if [[ -f "${BAG_DIR}/metadata.yaml" ]]; then
    printf 'Analyzing recorded trajectories...\n'
    docker exec -u "${CONTAINER_USER}" \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
      "${CONTAINER_NAME}" bash -lc '
        source /opt/ros/humble/setup.bash
        source /workspaces/isaac_ros-dev/install_docker/setup.bash
        python3 /workspaces/isaac_ros-dev/scripts/analyze_vslam_bag.py "$1"
      ' _ "${CONTAINER_BAG_DIR}" >"${LOG_DIR}/bag_analysis.log" 2>&1 || {
        printf 'Bag analysis failed; see %s\n' "${LOG_DIR}/bag_analysis.log" >&2
        return
      }
    printf 'Bag: %s\n' "${BAG_DIR}"
    printf 'Analysis: %s/analysis.md\n' "${BAG_DIR}"
  else
    printf 'Rosbag metadata was not created; see %s/rosbag.log\n' "${LOG_DIR}" >&2
  fi
}

cleanup() {
  if (( CLEANING_UP )); then
    return
  fi
  CLEANING_UP=1
  trap - EXIT INT TERM
  set +e

  printf '\nStopping mapping stack...\n'

  # Let rosbag receive a clean SIGINT and write metadata before its topics or
  # container disappear.
  if (( BAG_STARTED )) && is_container_running; then
    printf 'Capturing a 5-second stationary tail for drift measurement...\n'
    sleep 5
  fi
  finalize_bag

  for pid in "${HOST_PIDS[@]}"; do
    # Each host launch runs in its own session, so signal its complete process
    # group rather than leaving camera/bridge child nodes behind.
    kill -INT -- "-${pid}" >/dev/null 2>&1 || true
  done

  if is_container_running; then
    if (( CONTAINER_STARTED_BY_US )); then
      docker stop --time 5 "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    else
      stop_container_process "${CONTAINER_RVIZ_PID_FILE}" INT
      stop_container_process "${CONTAINER_VSLAM_PID_FILE}" INT
    fi
  fi

  sleep 1
  for pid in "${HOST_PIDS[@]}"; do
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  done

  if is_container_running && (( ! CONTAINER_STARTED_BY_US )); then
    stop_container_process "${CONTAINER_RVIZ_PID_FILE}" TERM
    stop_container_process "${CONTAINER_VSLAM_PID_FILE}" TERM
    docker exec "${CONTAINER_NAME}" rm -f \
      "${CONTAINER_RVIZ_PID_FILE}" "${CONTAINER_VSLAM_PID_FILE}" "${CONTAINER_BAG_PID_FILE}" \
      >/dev/null 2>&1 || true
  fi

  rm -f "${LAUNCHER_PID_FILE}"
  printf 'Stopped. Logs: %s\n' "${LOG_DIR}"
}

trap cleanup EXIT INT TERM

if [[ -f "${LAUNCHER_PID_FILE}" ]]; then
  existing_pid="$(<"${LAUNCHER_PID_FILE}")"
  if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    printf 'Mapping stack is already managed by PID %s.\n' "${existing_pid}" >&2
    exit 1
  fi
  rm -f "${LAUNCHER_PID_FILE}"
fi
printf '%s\n' "$$" > "${LAUNCHER_PID_FILE}"

if [[ ! -t 0 ]]; then
  printf 'Run this script from an interactive terminal for arrow-key input.\n' >&2
  exit 1
fi

for required_path in \
  "/opt/ros/humble/setup.bash" \
  "/home/maze/stm32_bridge_install/setup.bash" \
  "${REPO_ROOT}/install/setup.bash" \
  "${MAPPING_DOCKERFILE}"; do
  if [[ ! -e "${required_path}" ]]; then
    printf 'Required file is missing: %s\n' "${required_path}" >&2
    exit 1
  fi
done

if ! command -v setsid >/dev/null 2>&1; then
  printf 'Required command is missing: setsid\n' >&2
  exit 1
fi

source_with_nounset_disabled /opt/ros/humble/setup.bash

printf 'Logs: %s\n' "${LOG_DIR}"

if ! docker image inspect "${MAPPING_IMAGE}" >/dev/null 2>&1; then
  printf '[0/7] Building the persistent VSLAM mapping image (one time)...\n'
  docker build \
    --file "${MAPPING_DOCKERFILE}" \
    --tag "${MAPPING_IMAGE}" \
    "${REPO_ROOT}"
fi

printf '[1/7] Starting STM32 bridge...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  source /home/maze/stm32_bridge_install/setup.bash
  exec ros2 launch stm32_bridge stm32_bridge.launch.py \\
    transport:=i2c \\
    i2c_device:='${STM32_I2C_DEVICE}' \\
    i2c_address:='${STM32_I2C_ADDRESS}' \\
    i2c_poll_hz:='${STM32_I2C_POLL_HZ}' \\
    i2c_write_enabled:='${STM32_I2C_WRITE_ARG}' \\
    namespace:=leader
" >"${LOG_DIR}/stm32_bridge.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[2/7] Starting RealSense...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  # single shared D435 source for VSLAM and future survivor pipeline.
  exec ros2 launch realsense2_camera rs_launch.py \\
    camera_namespace:=leader camera_name:=camera \\
    enable_color:=true enable_depth:=true \\
    enable_infra:=true enable_infra1:=true enable_infra2:=true \\
    enable_sync:=true align_depth.enable:=true \\
    enable_gyro:=false enable_accel:=false \\
    publish_tf:=true tf_publish_rate:=30.0
" >"${LOG_DIR}/realsense.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[3/7] Starting Leader command selector in TELEOP mode...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  source '${REPO_ROOT}/install/setup.bash'
  exec ros2 launch leader_command_selector command_selector.launch.py \\
    source_mode:=TELEOP
" >"${LOG_DIR}/leader_command_selector.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[4/7] Preparing Isaac ROS container...\n'
if ! is_container_running; then
  if [[ -z "${DISPLAY:-}" ]] || ! command -v gnome-terminal >/dev/null 2>&1; then
    printf 'A graphical terminal is required to start the Isaac ROS container.\n' >&2
    exit 1
  fi
  CONTAINER_STARTED_BY_US=1
  gnome-terminal \
    --title="Isaac ROS container (managed by mapping launcher)" \
    -- bash -lc "
      export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
      docker run -it --rm \\
        --privileged \\
        --network host \\
        --ipc host \\
        --pid host \\
        --runtime nvidia \\
        --name '${CONTAINER_NAME}' \\
        --workdir /workspaces/isaac_ros-dev \\
        --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \\
        -e DISPLAY='${DISPLAY}' \\
        -e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all \\
        -e NVIDIA_DRIVER_CAPABILITIES=all \\
        -e ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' \\
        -e USER='${USER}' \\
        -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \\
        -e HOST_USER_UID='$(id -u)' \\
        -e HOST_USER_GID='$(id -g)' \\
        -v /tmp/.X11-unix:/tmp/.X11-unix \\
        -v /tmp/:/tmp/ \\
        -v '${HOME}/.Xauthority:/home/admin/.Xauthority:rw' \\
        -v '${REPO_ROOT}:/workspaces/isaac_ros-dev' \\
        -v /etc/localtime:/etc/localtime:ro \\
        -v /usr/bin/tegrastats:/usr/bin/tegrastats \\
        -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \\
        -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api \\
        -v /usr/share/vpi3:/usr/share/vpi3 \\
        -v /dev/input:/dev/input \\
        '${MAPPING_IMAGE}' /bin/bash
    "
fi

container_deadline=$((SECONDS + 300))
until is_container_running; do
  if (( SECONDS >= container_deadline )); then
    printf 'Timed out waiting for container %s.\n' "${CONTAINER_NAME}" >&2
    exit 1
  fi
  sleep 1
done

container_user_deadline=$((SECONDS + 30))
while [[ -z "${CONTAINER_USER}" ]]; do
  CONTAINER_USER="$(
    docker exec "${CONTAINER_NAME}" getent passwd "$(id -u)" 2>/dev/null \
      | cut -d: -f1 \
      || true
  )"
  if [[ -n "${CONTAINER_USER}" ]]; then
    break
  fi
  if ! is_container_running || (( SECONDS >= container_user_deadline )); then
    printf 'Container user initialization did not finish for %s.\n' "${CONTAINER_NAME}" >&2
    exit 1
  fi
  sleep 1
done
printf '  container user: %s\n' "${CONTAINER_USER}"

if ! docker exec -u "${CONTAINER_USER}" "${CONTAINER_NAME}" bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  ros2 pkg prefix rescue_robot_bringup >/dev/null
  ros2 pkg prefix isaac_ros_visual_slam >/dev/null
  ros2 pkg prefix nvblox_ros >/dev/null
'; then
  printf 'The container is missing a required ROS package or install_docker overlay.\n' >&2
  exit 1
fi
if [[ "${VSLAM_ONLY}" != "1" ]] && ! docker exec -u "${CONTAINER_USER}" "${CONTAINER_NAME}" bash -lc '
  source /opt/ros/humble/setup.bash
  ros2 pkg prefix nav2_planner >/dev/null
  ros2 pkg prefix nav2_controller >/dev/null
  ros2 pkg prefix nav2_bt_navigator >/dev/null
  ros2 pkg prefix nav2_lifecycle_manager >/dev/null
'; then
  printf 'The container is missing a required Nav2 package.\n' >&2
  exit 1
fi

container_log_dir="/workspaces/isaac_ros-dev/log/vslam_mapping_${RUN_ID}"
docker exec -d -u "${CONTAINER_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  -e DAMGC_VSLAM_HEADLESS="${VSLAM_HEADLESS}" \
  -e DAMGC_VSLAM_ONLY="${VSLAM_ONLY}" \
  "${CONTAINER_NAME}" bash -lc '
    log_path="$1"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:${LD_LIBRARY_PATH}"
    echo "$$" > /tmp/damgc_vslam_mapping_vslam.pid
    if [[ "${DAMGC_VSLAM_ONLY}" == "1" ]]; then
      exec ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py \
        publish_odom_to_base_tf:=true >>"${log_path}" 2>&1
    fi
    exec ros2 launch rescue_robot_bringup nvblox_vslam_realsense.launch.py >>"${log_path}" 2>&1
  ' _ "${container_log_dir}/vslam_nvblox.log"

if [[ "${VSLAM_HEADLESS}" == "1" ]]; then
  if [[ "${VSLAM_ONLY}" == "1" ]]; then
    printf '[5/7] VSLAM-only headless mode: nvblox and RViz are disabled...\n'
  else
    printf '[5/7] Headless mode: skipping RViz...\n'
  fi
else
  printf '[5/7] Starting RViz...\n'
  docker exec -d -u "${CONTAINER_USER}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    "${CONTAINER_NAME}" bash -lc '
      log_path="$1"
      source /opt/ros/humble/setup.bash
      source /workspaces/isaac_ros-dev/install_docker/setup.bash
      echo "$$" > /tmp/damgc_vslam_mapping_rviz.pid
      exec rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz >>"${log_path}" 2>&1
    ' _ "${container_log_dir}/rviz.log"
fi

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  until ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; do
    if (( SECONDS >= deadline )); then
      printf 'Timed out waiting for topic %s. Check %s\n' "${topic}" "${LOG_DIR}" >&2
      return 1
    fi
    sleep 1
  done
  printf '  ready: %s\n' "${topic}"
}

printf 'Waiting for the mapping data path...\n'
wait_for_topic "/leader/odom/raw" 30
wait_for_topic "/leader/camera/infra1/image_rect_raw" 45
wait_for_topic "/visual_slam/tracking/odometry" 120

printf '[6/7] Starting metrics rosbag...\n'
docker exec "${CONTAINER_NAME}" rm -f "${CONTAINER_BAG_PID_FILE}" >/dev/null 2>&1 || true
docker exec -d -u "${CONTAINER_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  "${CONTAINER_NAME}" bash -lc '
    bag_path="$1"
    log_path="$2"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    echo "$$" > /tmp/damgc_vslam_mapping_bag.pid
    exec ros2 bag record --output "${bag_path}" \
      /leader/cmd_vel \
      /leader/odom/raw \
      /leader/imu/data_raw \
      /leader/odometry/local \
      /leader/odometry/global \
      /visual_slam/tracking/odometry \
      /visual_slam/vis/slam_odometry \
      /visual_slam/slam_odometry_with_covariance \
      /visual_slam/status \
      /tf \
      /tf_static >>"${log_path}" 2>&1
  ' _ "${CONTAINER_BAG_DIR}" "${container_log_dir}/rosbag.log"
BAG_STARTED=1

bag_deadline=$((SECONDS + 15))
until container_process_running "${CONTAINER_BAG_PID_FILE}" && \
  grep -Fq 'Recording...' "${LOG_DIR}/rosbag.log" 2>/dev/null && \
  grep -Fq "Subscribed to topic '/visual_slam/tracking/odometry'" \
    "${LOG_DIR}/rosbag.log" 2>/dev/null; do
  if (( SECONDS >= bag_deadline )); then
    printf 'Rosbag failed to start. Check %s/rosbag.log\n' "${LOG_DIR}" >&2
    exit 1
  fi
  sleep 1
done
printf '  recording: %s\n' "${BAG_DIR}"
# /leader/cmd_vel is intentionally discovered after the teleop node starts.
# The sensor topics are already present, so the baseline delay also gives the
# recorder time to finish subscribing to their VSLAM/odometry streams.
printf '  capturing a 5-second stationary baseline...\n'
sleep 5

printf '[7/7] Starting arrow-key control. E/D changes speed; Space stops; Ctrl-C shuts everything down.\n'
source_with_nounset_disabled "${REPO_ROOT}/install/setup.bash"
ros2 run rescue_robot_bringup arrow_key_teleop.py --ros-args \
  -p command_topic:=/leader/teleop/cmd_vel \
  -p linear_speed:=0.08 \
  -p angular_speed:=0.25
