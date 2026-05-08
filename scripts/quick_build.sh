#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${WS_ROOT:-${REPO_ROOT}}"
source "${SCRIPT_DIR}/ros_env.sh"

if ! ROS_SETUP_BASH_FILE="$(resolve_ros_setup_bash)"; then
  echo "Unable to resolve ROS setup script. Set ROS_DISTRO or ROS_SETUP_BASH."
  exit 1
fi

echo "Workspace: ${WS_ROOT}"
echo "ROS setup: ${ROS_SETUP_BASH_FILE}"
echo "Removing old build artifacts..."
rm -rf "${WS_ROOT}/build" "${WS_ROOT}/install" "${WS_ROOT}/log"

source_bash_file_allow_unset_vars "${ROS_SETUP_BASH_FILE}"

echo "Starting full rebuild..."
cd "${WS_ROOT}"
colcon build --symlink-install