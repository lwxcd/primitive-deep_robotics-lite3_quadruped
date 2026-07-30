#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_WS="$PKG/src/Lite3_ROS"

# ROS setup scripts may reference unset variables.
set +u
source "$ROS_WS/install/setup.bash"
set -u

exec ros2 launch transfer transfer_launch.py use_sim_time:=false
