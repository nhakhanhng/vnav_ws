#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_traj.sh — Run the reference trajectory publisher (circle, r=5 m, z=2 m).
#
#   scripts/run_traj.sh
# -----------------------------------------------------------------------------
source "$(dirname "${BASH_SOURCE[0]}")/env.sh" || exit 1
USE_SIM_TIME="${USE_SIM_TIME:-true}"
exec ros2 run controller_pkg traj_publisher --ros-args -p use_sim_time:="$USE_SIM_TIME"
