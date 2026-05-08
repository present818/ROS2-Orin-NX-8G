#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${WS_ROOT:-${REPO_ROOT}}"

WORLD_NAME="${WORLD_NAME:-robocup_home}"
GUI_MODE="${GUI_MODE:-false}"
MACHINE_TYPE_VALUE="${MACHINE_TYPE_VALUE:-ROSOrin_Mecanum}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${WS_ROOT}/simulations/robot_gazebo/rviz/nav_nav2.rviz}"
RVIZ_DELAY="${RVIZ_DELAY:-5}"
QT_FONT_DPI_VALUE="${QT_FONT_DPI_VALUE:-144}"

if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "Workspace not built yet: ${WS_ROOT}/install/setup.bash not found"
  echo "Run: cd \"${WS_ROOT}\" && colcon build --symlink-install --packages-up-to robot_gazebo"
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${WS_ROOT}/install/setup.bash"

export MACHINE_TYPE="${MACHINE_TYPE_VALUE}"

echo "Workspace: ${WS_ROOT}"
echo "World: ${WORLD_NAME}"
echo "Gazebo GUI: ${GUI_MODE}"
echo "Machine type: ${MACHINE_TYPE}"
echo "RViz config: ${RVIZ_CONFIG}"

ros2 launch robot_gazebo worlds.launch.py \
  world_name:="${WORLD_NAME}" \
  gui:="${GUI_MODE}" \
  machine_type:="${MACHINE_TYPE}" &
GAZEBO_PID=$!

sleep "${RVIZ_DELAY}"

unset QT_SCALE_FACTOR QT_AUTO_SCREEN_SCALE_FACTOR QT_SCREEN_SCALE_FACTORS
QT_FONT_DPI="${QT_FONT_DPI_VALUE}" rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!

cleanup() {
  kill "${RVIZ_PID}" "${GAZEBO_PID}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

wait "${GAZEBO_PID}"
