#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/lib/bridge_env.sh"
bridge_env_load "${ROOT_DIR}"

REMOTE_PROJECT="/home/unitree/go2_navigation_project"
RELEASE_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$-${RANDOM}"
LOCAL_ARCHIVE="$(mktemp "${TMPDIR:-/tmp}/go2-navigation-${RELEASE_ID}-XXXXXX.tar.gz")"
REMOTE_ARCHIVE="/tmp/go2-navigation-${RELEASE_ID}.tar.gz"

cleanup_local() {
  rm -f "${LOCAL_ARCHIVE}"
}
trap cleanup_local EXIT

tar -C "${ROOT_DIR}" \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  -czf "${LOCAL_ARCHIVE}" \
  go2_navigation go2_scripts

scp -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${LOCAL_ARCHIVE}" \
  "${GO2_USER}@${GO2_HOST}:${REMOTE_ARCHIVE}"

ssh -o StrictHostKeyChecking=accept-new \
  -J "${PI_USER}@${PI_HOST}" \
  "${GO2_USER}@${GO2_HOST}" \
  bash -s -- "${REMOTE_PROJECT}" "${REMOTE_ARCHIVE}" "${RELEASE_ID}" <<'REMOTE_SCRIPT'
set -euo pipefail

remote_project="$1"
remote_archive="$2"
release_id="$3"

if [[ ! "${release_id}" =~ ^[0-9]{8}T[0-9]{6}Z-[0-9]+-[0-9]+$ ]]; then
  echo "Invalid deployment release identifier: ${release_id}" >&2
  exit 1
fi

stage="${remote_project}.stage-${release_id}"
backup="${remote_project}.backup-${release_id}"
failed="${remote_project}.failed-${release_id}"
previous="${remote_project}.previous"
switched=false
had_previous=false

cleanup_remote() {
  local status=$?

  rm -f "${remote_archive}"
  if [[ ${status} -ne 0 && "${switched}" == "true" ]]; then
    echo "Deployment failed after activation; restoring the previous workspace." >&2
    if [[ -e "${remote_project}" || -L "${remote_project}" ]]; then
      mv "${remote_project}" "${failed}"
    fi
    if [[ "${had_previous}" == "true" ]]; then
      if [[ -e "${backup}" || -L "${backup}" ]]; then
        mv "${backup}" "${remote_project}"
      elif [[ -e "${previous}" || -L "${previous}" ]]; then
        mv "${previous}" "${remote_project}"
      fi
    fi
    rm -rf "${failed}"
  elif [[
    ${status} -ne 0
    && "${had_previous}" == "true"
    && ! -e "${remote_project}"
    && ! -L "${remote_project}"
    && ( -e "${backup}" || -L "${backup}" )
  ]]; then
    mv "${backup}" "${remote_project}"
  fi

  rm -rf "${stage}"
  if [[ ${status} -eq 0 ]]; then
    rm -rf "${backup}"
  fi

  trap - EXIT
  exit "${status}"
}
trap cleanup_remote EXIT

# These names include a validated, unique release identifier and are never the
# active workspace path, so cleaning a stale interrupted attempt is safe.
rm -rf "${stage}" "${backup}" "${failed}"
mkdir -p "${stage}/upload" "${stage}/src" "${stage}/scripts"
tar --warning=no-timestamp -C "${stage}/upload" -xzf "${remote_archive}"
mv "${stage}/upload/go2_navigation" "${stage}/src/go2_navigation"
cp "${stage}/upload/go2_scripts/"*.sh "${stage}/scripts/"
rm -rf "${stage}/upload"
chmod +x "${stage}/scripts/"*.sh
find "${stage}/src/go2_navigation/go2_navigation" -name '*.py' -exec chmod +x {} \;

# Build and resolve the package entirely in staging. The active workspace is
# untouched unless both steps succeed.
(
  cd "${stage}"
  set +u
  source /opt/ros/foxy/setup.bash
  source "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash"
  set -u
  python3 -m compileall -q "${stage}/src/go2_navigation"
  colcon build --packages-select go2_navigation
  test -f "${stage}/install/setup.bash"
  set +u
  source "${stage}/install/setup.bash"
  set -u
  package_prefix="$(ros2 pkg prefix go2_navigation)"
  [[ "${package_prefix}" == "${stage}/install/go2_navigation" ]]
)

if [[ -e "${remote_project}" || -L "${remote_project}" ]]; then
  mv "${remote_project}" "${backup}"
  had_previous=true
fi

if ! mv "${stage}" "${remote_project}"; then
  if [[ "${had_previous}" == "true" ]]; then
    mv "${backup}" "${remote_project}"
  fi
  exit 1
fi
switched=true

# Verify the activated path in a fresh shell environment. Any failure triggers
# the EXIT trap above, which restores the previous workspace.
(
  set +u
  source /opt/ros/foxy/setup.bash
  source "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash"
  source "${remote_project}/install/setup.bash"
  set -u
  package_prefix="$(ros2 pkg prefix go2_navigation)"
  [[ "${package_prefix}" == "${remote_project}/install/go2_navigation" ]]
  ros2 launch go2_navigation go2_navigation_stack.launch.py --show-args >/dev/null
  ros2 launch go2_navigation go2_live_mapping_stack.launch.py --show-args >/dev/null
  test -x "${remote_project}/scripts/start_navigation_with_rviz_bridge.sh"
  test -x "${remote_project}/scripts/start_live_mapping_with_rviz_bridge.sh"
)

if [[ "${had_previous}" == "true" ]]; then
  # Keep exactly one known-good release for an operator-initiated rollback.
  # The just-replaced workspace remains in ${backup} until activation checks
  # have passed, so replacing an older .previous copy is safe here.
  rm -rf "${previous}"
  mv "${backup}" "${previous}"
fi
switched=false
had_previous=false
echo "Activated GO2 navigation release ${release_id} at ${remote_project}"
if [[ -e "${previous}" || -L "${previous}" ]]; then
  echo "Previous verified workspace retained at ${previous}"
fi
REMOTE_SCRIPT

echo "Installed and verified GO2 navigation project at ${REMOTE_PROJECT}"
