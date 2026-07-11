#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lite3_quadruped runtime — start the ROS-aware primitive provider.
set -euo pipefail

PKG="$(cd "$(dirname "$0")/.." && pwd)"
# ROS setup.bash may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$PKG/src/Lite3_ROS/install/setup.bash" ]; then
  source "$PKG/src/Lite3_ROS/install/setup.bash"
fi
set -u

cleanup() {
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$PKG:$PYTHONPATH"
export PYTHONPATH="/home/jetson/cxk/robonix/pylib/robonix-api:$PYTHONPATH"

exec python3 -m lite3_quadruped.main
