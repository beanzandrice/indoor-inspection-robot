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

REMOTE_PROJECT="/home/unitree/go2_navigation_project"
ARCHIVE="/tmp/go2_navigation_project.tar.gz"

tar -C "${ROOT_DIR}" \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  -czf "${ARCHIVE}" \
  go2_navigation go2_scripts

ssh -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${GO2_USER}@${GO2_HOST}" \
  "mkdir -p '${REMOTE_PROJECT}/src' '${REMOTE_PROJECT}/scripts'"

scp -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${ARCHIVE}" \
  "${GO2_USER}@${GO2_HOST}:/tmp/go2_navigation_project.tar.gz"

ssh -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${GO2_USER}@${GO2_HOST}" \
  "set -eo pipefail
   rm -rf /tmp/go2_navigation_upload
   mkdir -p /tmp/go2_navigation_upload
   tar --warning=no-timestamp -C /tmp/go2_navigation_upload -xzf /tmp/go2_navigation_project.tar.gz
   rm -rf '${REMOTE_PROJECT}/src/go2_navigation'
   cp -r /tmp/go2_navigation_upload/go2_navigation '${REMOTE_PROJECT}/src/go2_navigation'
   cp /tmp/go2_navigation_upload/go2_scripts/*.sh '${REMOTE_PROJECT}/scripts/'
   chmod +x '${REMOTE_PROJECT}/scripts/'*.sh
   find '${REMOTE_PROJECT}/src/go2_navigation/go2_navigation' -name '*.py' -exec chmod +x {} \;
   cd '${REMOTE_PROJECT}'
   source /opt/ros/foxy/setup.bash
   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
   set -u
   colcon build --symlink-install --packages-select go2_navigation"

rm -f "${ARCHIVE}"

echo "Installed and rebuilt GO2 navigation project at ${REMOTE_PROJECT}"
