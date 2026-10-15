#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# env.sh — Source ROS 2 + this workspace. Safe to `source` from any directory.
#
#   source scripts/env.sh
#
# Exports:  WS_DIR, SCRIPTS_DIR
# -----------------------------------------------------------------------------

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPTS_DIR")"
export SCRIPTS_DIR WS_DIR

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"

if [ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
else
  echo "[env] ERROR: /opt/ros/${ROS_DISTRO_NAME}/setup.bash not found" >&2
  return 1 2>/dev/null || exit 1
fi

# Gazebo Classic resource paths (models like ground_plane / sun).
if [ -f /usr/share/gazebo/setup.sh ]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo/setup.sh
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "[env] WARNING: ${WS_DIR}/install/setup.bash not found — run scripts/build.sh first" >&2
fi
