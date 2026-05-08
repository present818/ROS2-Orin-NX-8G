#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SLAM_MODE=true
exec "${SCRIPT_DIR}/sim.sh" "$@"
