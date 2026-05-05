#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/lib/bridge_env.sh"
bridge_env_load "${ROOT_DIR}"

echo "Opening SSH tunnels through ${PI_USER}@${PI_HOST} to ${GO2_USER}@${GO2_HOST}"
echo "You should be prompted for the Pi password, then the GO2 password."
echo "Leave this terminal open. Press Ctrl+C here to close the tunnels."
echo "If you see 'connect_to 127.0.0.1 port 16000: failed', the laptop receiver is not listening yet."

exec ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  -L "${LAPTOP_RETURN_HOST}:${LAPTOP_RETURN_PORT}:${GO2_RECV_HOST}:${GO2_RECV_PORT}" \
  -R "${GO2_SEND_HOST}:${GO2_SEND_PORT}:${LAPTOP_RECV_HOST}:${LAPTOP_RECV_PORT}" \
  "${GO2_USER}@${GO2_HOST}"
