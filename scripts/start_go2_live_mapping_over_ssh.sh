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
  "GO2_RVIZ_SEND_HOST='${GO2_SEND_HOST}' GO2_RVIZ_SEND_PORT='${GO2_SEND_PORT}' GO2_RVIZ_RECV_HOST='${GO2_RECV_HOST}' GO2_RVIZ_RECV_PORT='${GO2_RECV_PORT}' GO2_RVIZ_TF_MAX_HZ='${GO2_RVIZ_TF_MAX_HZ:-20.0}' GO2_RVIZ_IMAGE_MAX_HZ='${GO2_RVIZ_IMAGE_MAX_HZ:-0.0}' GO2_RVIZ_POINTCLOUD_MAX_HZ='${GO2_RVIZ_POINTCLOUD_MAX_HZ:-0.5}' GO2_CAMERA_ENABLE='${GO2_CAMERA_ENABLE:-false}' GO2_CAMERA_FPS='${GO2_CAMERA_FPS:-${GO2_VIDEO_FPS:-5}}' GO2_JOINT_TF_ENABLE='${GO2_JOINT_TF_ENABLE:-true}' GO2_LOWSTATE_TOPICS='${GO2_LOWSTATE_TOPICS:-lowstate,lf/lowstate}' GO2_JOINT_TF_HZ='${GO2_JOINT_TF_HZ:-20.0}' GO2_VIDEO_ENABLE='false' /home/unitree/go2_navigation_project/scripts/start_live_mapping_with_rviz_bridge.sh"
