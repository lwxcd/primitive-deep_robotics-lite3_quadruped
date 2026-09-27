#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lite3_quadruped runtime — start the ROS-aware primitive provider.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

if [[ "${ROBONIX_LITE3_MODE:-hardware}" == "simulation" ]]; then
  container="${LITE3_SIM_CONTAINER:-lite3-sim}"
  docker inspect "$container" >/dev/null 2>&1 || {
    echo "simulation container is not running: $container" >&2
    exit 1
  }
  exec docker exec -i \
    -e RBNX_PACKAGE_ROOT=/workspace/primitive-deep_robotics-lite3_quadruped \
    -e RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-lite3_quadruped}" \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "$container" bash -lc '
      source /opt/ros/humble/setup.bash
      export PYTHONPATH=/workspace/robonix/pylib/robonix-api:/workspace/primitive-deep_robotics-lite3_quadruped:${PYTHONPATH:-}
      cd /workspace/primitive-deep_robotics-lite3_quadruped
      exec python3 -m lite3_quadruped.main
    '
fi

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

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

exec python3 -m lite3_quadruped.main
