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

WORLD_NAME="${WORLD_NAME:-robocup_home}"
GUI_MODE="${GUI_MODE:-false}"
SLAM_MODE="${SLAM_MODE:-false}"
MACHINE_TYPE_VALUE="${MACHINE_TYPE_VALUE:-ROSOrin_Mecanum}"
RVIZ_DELAY="${RVIZ_DELAY:-5}"
RVIZ_ENABLE="${RVIZ_ENABLE:-auto}"
QT_FONT_DPI_VALUE="${QT_FONT_DPI_VALUE:-340}"
LOG_DIR="${WS_ROOT}/simulations/logs"
LOG_TIMESTAMP="$(date +"%Y-%m-%d_%H-%M-%S")"
RUN_LOG_DIR="${LOG_DIR}/sim_${LOG_TIMESTAMP}"
LOG_FILE="${RUN_LOG_DIR}/terminal.log"
ROS_LOG_DIR="${ROS_LOG_DIR:-${RUN_LOG_DIR}/ros}"
MAP_FILE="${MAP_FILE:-${WS_ROOT}/install/robot_gazebo/share/robot_gazebo/maps/map_01.yaml}"

mkdir -p "${LOG_DIR}"
mkdir -p "${RUN_LOG_DIR}"
mkdir -p "${ROS_LOG_DIR}"

exec > >(tee -a "${LOG_FILE}") 2>&1

mapfile -t EXISTING_RUN_LOG_DIRS < <(find "${LOG_DIR}" -mindepth 1 -maxdepth 1 -type d -name 'sim_*' -printf '%f\n' | sort)
if (( ${#EXISTING_RUN_LOG_DIRS[@]} > 5 )); then
  REMOVE_COUNT=$(( ${#EXISTING_RUN_LOG_DIRS[@]} - 5 ))
  for log_dir_name in "${EXISTING_RUN_LOG_DIRS[@]:0:${REMOVE_COUNT}}"; do
    rm -rf "${LOG_DIR:?}/${log_dir_name}"
  done
fi

echo "Log dir: ${RUN_LOG_DIR}"
echo "Log file: ${LOG_FILE}"
echo "ROS setup: ${ROS_SETUP_BASH_FILE}"

if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "Workspace not built yet: ${WS_ROOT}/install/setup.bash not found"
  echo "Run: cd \"${WS_ROOT}\" && colcon build --symlink-install --packages-up-to robot_gazebo"
  exit 1
fi

source_bash_file_allow_unset_vars "${ROS_SETUP_BASH_FILE}"
source_bash_file_allow_unset_vars "${WS_ROOT}/install/setup.bash"

append_unique_path() {
  local current_value="${1:-}"
  local new_path="${2:-}"

  if [[ -z "${new_path}" || ! -d "${new_path}" ]]; then
    printf '%s' "${current_value}"
    return
  fi

  case ":${current_value}:" in
    *":${new_path}:"*) printf '%s' "${current_value}" ;;
    :)
      printf '%s' "${new_path}"
      ;;
    *)
      printf '%s:%s' "${new_path}" "${current_value}"
      ;;
  esac
}

RESOURCE_SHARE_ROOTS=(
  "${WS_ROOT}/install/robot_gazebo/share"
  "${WS_ROOT}/install/rosorin_description/share"
)

for resource_root in "${RESOURCE_SHARE_ROOTS[@]}"; do
  GZ_SIM_RESOURCE_PATH="$(append_unique_path "${GZ_SIM_RESOURCE_PATH:-}" "${resource_root}")"
  IGN_GAZEBO_RESOURCE_PATH="$(append_unique_path "${IGN_GAZEBO_RESOURCE_PATH:-}" "${resource_root}")"
done

export GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH
export ROS_LOG_DIR
export MACHINE_TYPE="${MACHINE_TYPE_VALUE}"
export need_compile="True"

SIM_MODE="nav"
if [[ "${SLAM_MODE}" == "true" ]]; then
  SIM_MODE="slam"
fi

NAV_LAUNCH_FILE="${WS_ROOT}/install/robot_gazebo/share/robot_gazebo/launch/include/navigation.launch.py"
SLAM_LAUNCH_FILE="${WS_ROOT}/install/robot_gazebo/share/robot_gazebo/launch/include/slam.launch.py"

echo "Workspace: ${WS_ROOT}"
echo "World: ${WORLD_NAME}"
echo "Sim mode: ${SIM_MODE}"
echo "Gazebo GUI: ${GUI_MODE}"
echo "Machine type: ${MACHINE_TYPE}"
echo "ROS log dir: ${ROS_LOG_DIR}"
if [[ "${SIM_MODE}" == "nav" ]]; then
  echo "Map file: ${MAP_FILE}"
fi

ros2 launch robot_gazebo worlds.launch.py \
  world_name:="${WORLD_NAME}" \
  gui:="${GUI_MODE}" \
  machine_type:="${MACHINE_TYPE}" &
GAZEBO_PID=$!

RVIZ_PID=""
STACK_PID=""

sleep "${RVIZ_DELAY}"

if ! kill -0 "${GAZEBO_PID}" 2>/dev/null; then
  wait "${GAZEBO_PID}"
  exit $?
fi

unset QT_SCALE_FACTOR QT_AUTO_SCREEN_SCALE_FACTOR QT_SCREEN_SCALE_FACTORS

START_RVIZ=false
case "${RVIZ_ENABLE}" in
  true)
    START_RVIZ=true
    ;;
  false)
    START_RVIZ=false
    ;;
  auto)
    if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
      START_RVIZ=true
    fi
    ;;
  *)
    echo "Invalid RVIZ_ENABLE=${RVIZ_ENABLE}; expected true, false, or auto"
    exit 1
    ;;
esac

echo "Launch RViz: ${START_RVIZ}"

if [[ "${SIM_MODE}" == "nav" ]]; then
  if [[ ! -f "${MAP_FILE}" ]]; then
    echo "Map file not found: ${MAP_FILE}"
    exit 1
  fi
  ros2 launch "${NAV_LAUNCH_FILE}" \
    use_sim_time:=true \
    map:="${MAP_FILE}" \
    launch_rviz:="${START_RVIZ}" &
  STACK_PID=$!
else
  ros2 launch "${SLAM_LAUNCH_FILE}" \
    use_sim_time:=true \
    enable_save:=false \
    launch_rviz:="${START_RVIZ}" &
  STACK_PID=$!
fi

cleanup() {
  if [[ -n "${STACK_PID}" ]]; then
    kill "${STACK_PID}" 2>/dev/null || true
  fi
  kill "${GAZEBO_PID}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

wait -n "${GAZEBO_PID}" "${STACK_PID}"
