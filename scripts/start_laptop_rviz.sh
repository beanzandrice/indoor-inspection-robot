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

LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"
RECEIVER_LOG="${LOG_DIR}/laptop_rviz_receiver.log"

python3 "${ROOT_DIR}/scripts/laptop_rviz_receiver.py" \
  --listen-host "${LAPTOP_RECV_HOST}" \
  --recv-port "${LAPTOP_RECV_PORT}" \
  --robot-host "${LAPTOP_RETURN_HOST}" \
  --send-port "${LAPTOP_RETURN_PORT}" \
  > "${RECEIVER_LOG}" 2>&1 &
RECEIVER_PID=$!

sleep 1
if ! kill -0 "${RECEIVER_PID}" >/dev/null 2>&1; then
  echo "Laptop receiver exited during startup. Check ${RECEIVER_LOG}"
  exit 1
fi

cleanup() {
  kill "${RECEIVER_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Laptop receiver PID: ${RECEIVER_PID}"
echo "Receiver log: ${RECEIVER_LOG}"
"${RVIZ_BIN:-rviz2}" -d "${ROOT_DIR}/rviz/go2_navigation.rviz"
