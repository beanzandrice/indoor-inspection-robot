#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/config/bridge.env"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "Missing ${ENV_FILE}"
  echo "Copy config/bridge.env.example to config/bridge.env and edit it for your network."
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

export ROS_LOCALHOST_ONLY="${LAPTOP_ROS_LOCALHOST_ONLY:-1}"

LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"
RECEIVER_LOG="${LOG_DIR}/laptop_rviz_receiver.log"
MODEL_LOG="${LOG_DIR}/go2_robot_model_publisher.log"
GO2_URDF="${GO2_URDF:-${ROOT_DIR}/robot_description/go2/urdf/go2.urdf}"
ENABLE_GO2_MODEL="${ENABLE_GO2_MODEL:-true}"

MODEL_PID=""
if [[ "${ENABLE_GO2_MODEL}" == "true" ]]; then
  if [[ ! -f "${GO2_URDF}" ]]; then
    echo "Go2 URDF not found: ${GO2_URDF}"
    echo "Set GO2_URDF to a valid file or set ENABLE_GO2_MODEL=false."
    exit 1
  fi

  python3 "${ROOT_DIR}/scripts/go2_robot_model_publisher.py" \
    --urdf "${GO2_URDF}" \
    > "${MODEL_LOG}" 2>&1 &
  MODEL_PID=$!
fi

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

cleanup() {
  kill "${RECEIVER_PID}" >/dev/null 2>&1 || true
  if [[ -n "${MODEL_PID}" ]]; then
    kill "${MODEL_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 1
if [[ -n "${MODEL_PID}" ]] && ! kill -0 "${MODEL_PID}" >/dev/null 2>&1; then
  echo "Go2 robot model publisher exited during startup. Check ${MODEL_LOG}"
  exit 1
fi

if ! kill -0 "${RECEIVER_PID}" >/dev/null 2>&1; then
  echo "Laptop receiver exited during startup. Check ${RECEIVER_LOG}"
  exit 1
fi

if [[ -n "${MODEL_PID}" ]]; then
  echo "Go2 robot model PID: ${MODEL_PID}"
  echo "Go2 URDF: ${GO2_URDF}"
  echo "Robot model log: ${MODEL_LOG}"
fi
echo "Laptop receiver PID: ${RECEIVER_PID}"
echo "Receiver log: ${RECEIVER_LOG}"
"${RVIZ_BIN:-rviz2}" -d "${ROOT_DIR}/rviz/go2_navigation.rviz"
