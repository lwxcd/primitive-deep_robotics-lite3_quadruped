#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lite3_quadruped build — codegen + optional ROS2 package build.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[lite3_quadruped/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

ROS_SRC="$PKG/src/Lite3_ROS"
if [ -d "$ROS_SRC" ]; then
  if command -v colcon >/dev/null 2>&1; then
    echo "[lite3_quadruped/build] building ROS2 package in src/Lite3_ROS"
    pushd "$ROS_SRC" >/dev/null
    colcon build --merge-install
    popd >/dev/null
  else
    echo "[lite3_quadruped/build] warning: colcon not found, skipping ROS2 build"
  fi
fi

echo "[lite3_quadruped/build] done."
