#!/usr/bin/env bash

resolve_ros_setup_bash() {
  local candidate
  local -a candidates=()

  if [[ -n "${ROS_SETUP_BASH:-}" && -f "${ROS_SETUP_BASH}" ]]; then
    printf '%s\n' "${ROS_SETUP_BASH}"
    return 0
  fi

  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidate="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi

  if [[ -d /opt/ros ]]; then
    mapfile -t candidates < <(find /opt/ros -mindepth 2 -maxdepth 2 -type f -path '/opt/ros/*/setup.bash' | sort)
  fi

  if (( ${#candidates[@]} == 1 )); then
    printf '%s\n' "${candidates[0]}"
    return 0
  fi

  return 1
}

source_bash_file_allow_unset_vars() {
  local bash_file="${1:?bash file is required}"
  local had_nounset=0

  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "${bash_file}"

  if (( had_nounset )); then
    set -u
  fi
}