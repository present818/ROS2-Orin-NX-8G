#!/usr/bin/env bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/slam_logs"
MAIN_LOG="${LOG_DIR}/main.log"
SLAM_LOG="${LOG_DIR}/slam.log"
TELEOP_LOG="${LOG_DIR}/teleop.log"
RVIZ_LOG="${LOG_DIR}/rviz.log"
MAP_SAVE_LOG="${LOG_DIR}/map_save.log"
DIAG_LOG="${LOG_DIR}/diagnostics.log"
RVIZ_CONFIG="${HOME}/ros2_ws/slam/rviz/slam_desktop.rviz"

rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"
exec > >(tee "${MAIN_LOG}") 2>&1

echo "[slam.sh] $(date '+%F %T')"
echo "[slam.sh] HOME=${HOME}"
echo "[slam.sh] script_dir=${SCRIPT_DIR}"
echo "[slam.sh] log_dir=${LOG_DIR}"

pause_on_error() {
    echo
    echo "[slam.sh] startup failed; see logs in ${LOG_DIR}"
    read -r -p "Press Enter to close..."
}

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[slam.sh] gnome-terminal not found"
    pause_on_error
    exit 1
fi

if ! command -v zsh >/dev/null 2>&1; then
    echo "[slam.sh] zsh not found"
    pause_on_error
    exit 1
fi

if [ ! -r "${HOME}/.zshrc" ]; then
    echo "[slam.sh] missing ${HOME}/.zshrc"
    pause_on_error
    exit 1
fi

if [ ! -r "${RVIZ_CONFIG}" ]; then
    echo "[slam.sh] missing RViz config: ${RVIZ_CONFIG}"
    pause_on_error
    exit 1
fi

echo "[slam.sh] source workspace environment for diagnostics"
zsh -ic 'source "$HOME/.zshrc"; echo "[slam.sh] ros2=$(command -v ros2)"; echo "[slam.sh] slam prefix=$(ros2 pkg prefix slam 2>/dev/null || true)"; prefix=$(ros2 pkg prefix slam 2>/dev/null || true); if [ -n "$prefix" ]; then launch_file="$prefix/share/slam/launch/slam.launch.py"; echo "[slam.sh] installed slam launch=${launch_file}"; grep -n "ros2_ws/src" "$launch_file" 2>/dev/null || true; fi'

if ! zsh -ic 'source "$HOME/.zshrc"; ros2 pkg prefix slam >/dev/null 2>&1'; then
    echo "[slam.sh] ROS package 'slam' is not available in the current environment."
    echo "[slam.sh] The workspace overlay is probably not built or not sourced."
    echo "[slam.sh] Build it on the robot with:"
    echo "  cd ~/ros2_ws"
    echo "  colcon build --symlink-install --packages-select slam peripherals controller servo_controller ros_robot_controller rosorin_description"
    echo "  source ~/.zshrc"
    pause_on_error
    exit 1
fi

tab_cmd() {
    local command="$1"
    local log_file="$2"
    printf 'exec > >(tee "%s") 2>&1; echo "[slam.sh] tab started: $(date '"'"'+%%F %%T'"'"')"; source "$HOME/.zshrc"; %s; status=$?; echo; echo "[slam.sh] command exited with status ${status}"; exec zsh' "${log_file}" "${command}"
}

diagnostics_cmd() {
    local log_file="$1"
    printf 'exec > >(tee "%s") 2>&1; source "$HOME/.zshrc"; echo "[slam.sh] diagnostics waiting for graph..."; sleep 20; echo "[slam.sh] diagnostics: $(date '"'"'+%%F %%T'"'"')"; echo; echo "== ros2 node list =="; ros2 node list || true; echo; echo "== ros2 topic list =="; ros2 topic list || true; echo; echo "== ros2 service list | grep slam =="; ros2 service list | grep -E "slam|map|save" || true; echo; echo "== /tf_static sample =="; timeout 5 ros2 topic echo /tf_static --once || true; echo; echo "== /tf sample =="; timeout 5 ros2 topic echo /tf --once || true; echo; echo "== /scan_raw sample =="; timeout 5 ros2 topic echo /scan_raw --once || true; echo; echo "== /map sample =="; timeout 5 ros2 topic echo /map --once || true; echo; echo "[slam.sh] diagnostics finished"; exec zsh' "${log_file}"
}

gnome-terminal \
    --tab --title="SLAM" -- zsh -ic "$(tab_cmd 'sudo systemctl stop start_app_node.service; ros2 launch slam slam.launch.py enable_save:=false' "${SLAM_LOG}")" \
    --tab --title="Teleop" -- zsh -ic "$(tab_cmd 'sleep 10; ros2 launch peripherals teleop_key_control.launch.py' "${TELEOP_LOG}")" \
    --tab --title="RViz" -- zsh -ic "$(tab_cmd "sleep 10; rviz2 rviz2 -d ${RVIZ_CONFIG}" "${RVIZ_LOG}")" \
    --tab --title="Map Save" -- zsh -ic "$(tab_cmd 'sleep 10; ros2 run slam map_save' "${MAP_SAVE_LOG}")" \
    --tab --title="Diagnostics" -- zsh -ic "$(diagnostics_cmd "${DIAG_LOG}")"

status=$?
if [ "${status}" -ne 0 ]; then
    echo "[slam.sh] gnome-terminal exited with status ${status}"
    pause_on_error
    exit "${status}"
fi

echo "[slam.sh] gnome-terminal launched"
