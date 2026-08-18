#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/config/bridge.env"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-${HOME}/isaacsim}"
ISAAC_ROS_BRIDGE_ROOT="${ISAAC_ROS_BRIDGE_ROOT:-${ISAACSIM_ROOT}/exts/isaacsim.ros2.bridge/humble}"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "Missing ${ENV_FILE}"
  echo "Copy config/bridge.env.example to config/bridge.env and edit it for your network."
  exit 1
fi

if [[ ! -x "${ISAACSIM_ROOT}/python.sh" ]]; then
  echo "Isaac Sim python.sh not found or not executable: ${ISAACSIM_ROOT}/python.sh"
  echo "Set ISAACSIM_ROOT to your Isaac Sim install path."
  exit 1
fi

if [[ ! -d "${ISAAC_ROS_BRIDGE_ROOT}/rclpy" || ! -d "${ISAAC_ROS_BRIDGE_ROOT}/lib" ]]; then
  echo "Isaac Sim bundled ROS 2 Humble libraries not found under: ${ISAAC_ROS_BRIDGE_ROOT}"
  echo "Set ISAAC_ROS_BRIDGE_ROOT if your Isaac Sim ROS 2 bridge is installed elsewhere."
  exit 1
fi

source "${ENV_FILE}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source /opt/ros/foxy/setup.bash
else
  echo "Could not find /opt/ros/humble/setup.bash or /opt/ros/foxy/setup.bash"
  exit 1
fi
set -u

export ROS_DOMAIN_ID="${LAPTOP_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-25}}"
export ROS_LOCALHOST_ONLY="${LAPTOP_ROS_LOCALHOST_ONLY:-1}"

LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"
RECEIVER_LOG="${LOG_DIR}/laptop_rviz_receiver.log"
ISAAC_LOG="${LOG_DIR}/isaac_go2_live_nav_viewer.log"

port_in_use() {
  ss -ltn "( sport = :$1 )" 2>/dev/null | awk 'NR > 1 { found = 1 } END { exit found ? 0 : 1 }'
}

RECEIVER_PID=""
RECEIVER_OWNED=false

if port_in_use "${LAPTOP_RECV_PORT}"; then
  echo "Port ${LAPTOP_RECV_PORT} is already listening; assuming laptop_rviz_receiver.py is already running."
else
  python3 "${ROOT_DIR}/scripts/laptop_rviz_receiver.py" \
    --listen-host "${LAPTOP_RECV_HOST}" \
    --recv-port "${LAPTOP_RECV_PORT}" \
    --robot-host "${LAPTOP_RETURN_HOST}" \
    --send-port "${LAPTOP_RETURN_PORT}" \
    --protocol "${GO2_RVIZ_PROTOCOL:-binary}" \
    --max-frame-bytes "${GO2_RVIZ_MAX_FRAME_BYTES:-16777216}" \
    --max-buffer-bytes "${GO2_RVIZ_MAX_BUFFER_BYTES:-20971520}" \
    --write-timeout "${GO2_RVIZ_WRITE_TIMEOUT:-2.0}" \
    --command-max-age "${GO2_COMMAND_MAX_AGE:-5.0}" \
    > "${RECEIVER_LOG}" 2>&1 &
  RECEIVER_PID=$!
  RECEIVER_OWNED=true
fi

cleanup() {
  if [[ "${RECEIVER_OWNED}" == "true" && -n "${RECEIVER_PID}" ]]; then
    kill "${RECEIVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "${RECEIVER_OWNED}" == "true" ]]; then
  sleep 1
  if ! kill -0 "${RECEIVER_PID}" >/dev/null 2>&1; then
    echo "Laptop receiver exited during startup. Check ${RECEIVER_LOG}"
    exit 1
  fi
  echo "Laptop receiver PID: ${RECEIVER_PID}"
fi

echo "Receiver log: ${RECEIVER_LOG}"
echo "Isaac viewer log: ${ISAAC_LOG}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Starting Isaac Sim GO2 live navigation viewer..."

env \
  -u AMENT_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u RMW_IMPLEMENTATION \
  PYTHONUNBUFFERED=1 \
  PYTHONPATH="${ISAAC_ROS_BRIDGE_ROOT}/rclpy" \
  LD_LIBRARY_PATH="${ISAAC_ROS_BRIDGE_ROOT}/lib" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  "${ISAACSIM_ROOT}/python.sh" "${ROOT_DIR}/isaac/go2_live_nav_viewer.py" "$@" \
  2>&1 | tee "${ISAAC_LOG}"
