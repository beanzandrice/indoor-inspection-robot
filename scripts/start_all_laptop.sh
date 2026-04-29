#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Start scripts/start_tunnels.sh in another terminal first and leave it open."
echo "Then this script starts the laptop receiver and RViz2."
"${ROOT_DIR}/scripts/start_laptop_rviz.sh"
