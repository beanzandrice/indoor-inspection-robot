#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/config/bridge.env"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "Missing ${ENV_FILE}"
  echo "Copy config/bridge.env.example to config/bridge.env and edit it for your network."
  exit 1
fi

source "${ENV_FILE}"

ssh -t \
  -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${GO2_USER}@${GO2_HOST}" \
  "GO2_RVIZ_SEND_HOST='${GO2_SEND_HOST}' GO2_RVIZ_SEND_PORT='${GO2_SEND_PORT}' GO2_RVIZ_RECV_HOST='${GO2_RECV_HOST}' GO2_RVIZ_RECV_PORT='${GO2_RECV_PORT}' GO2_RVIZ_IMAGE_MAX_HZ='${GO2_RVIZ_IMAGE_MAX_HZ:-5.0}' GO2_RVIZ_POINTCLOUD_MAX_HZ='${GO2_RVIZ_POINTCLOUD_MAX_HZ:-2.0}' GO2_VIDEO_ENABLE='${GO2_VIDEO_ENABLE:-true}' GO2_VIDEO_FPS='${GO2_VIDEO_FPS:-10}' /home/unitree/go2_navigation_project/scripts/start_navigation_with_rviz_bridge.sh"
