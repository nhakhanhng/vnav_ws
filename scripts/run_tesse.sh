#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_tesse.sh — Full Lab 3 stack on the TESSE (Unity) simulator, one tmux
# window per component:
#
#   0 sim         Unity player   vnav_builds/lab3/lab3.x86_64
#   1 bridge      tesse_ros_bridge (tesse_ros_node + quadrotor_control_interface)
#   2 traj        traj_publisher        -> /desired_state
#   3 controller  controller_node       -> /rotor_speed_cmds
#   4 foxglove    (optional) markers + foxglove_bridge on ws://localhost:8765
#   5 rviz        (optional) rviz2
#
# Usage:
#   scripts/run_tesse.sh [options]
#
# Options:
#   --foxglove         also start Foxglove visualization
#   --rviz             also start RViz2
#   --no-sim           do not start Unity (you already started it)
#   --no-controller    do not start controller_node (e.g. run it by hand)
#   --sim-wait N       seconds to wait for Unity before starting the bridge (default 8)
#   --detach           start in background without attaching to tmux
#   -h, --help         show this help
#
# Stop everything:  scripts/stop.sh tesse
# -----------------------------------------------------------------------------
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "${SCRIPTS_DIR}/_tmux_lib.sh"

SESSION="vnav_tesse"
FOXGLOVE=0; RVIZ=0; SIM=1; CONTROLLER=1; DETACH=0; SIM_WAIT=8

while [ $# -gt 0 ]; do
  case "$1" in
    --foxglove)      FOXGLOVE=1 ;;
    --rviz)          RVIZ=1 ;;
    --no-sim)        SIM=0 ;;
    --no-controller) CONTROLLER=0 ;;
    --sim-wait)
      [ $# -ge 2 ] || { echo "ERROR: --sim-wait requires a number of seconds" >&2; exit 1; }
      SIM_WAIT="$2"; shift ;;
    --detach)        DETACH=1 ;;
    -h|--help)       awk 'NR>1 && /^#/{sub(/^# ?/,"");print;next} NR>1{exit}' "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 1 ;;
  esac
  shift
done

[[ "$SIM_WAIT" =~ ^[0-9]{1,4}$ ]] || {
  echo "ERROR: --sim-wait must be an integer from 0 to 9999 seconds" >&2; exit 1; }
SIM_WAIT=$((10#$SIM_WAIT))
ros2 pkg prefix tesse_ros_bridge >/dev/null 2>&1 || {
  echo "ERROR: tesse_ros_bridge not built. Run: scripts/build.sh tesse_ros_bridge" >&2; exit 1; }
if [ "$FOXGLOVE" = "1" ]; then
  ros2 pkg prefix tesse_foxglove_pkg >/dev/null 2>&1 || {
    echo "ERROR: tesse_foxglove_pkg not built. Run: scripts/build.sh tesse_foxglove_pkg" >&2; exit 1; }
fi

SIM_BIN="${WS_DIR}/vnav_builds/lab3/lab3.x86_64"
if [ "$SIM" = "1" ]; then
  [ -x "$SIM_BIN" ] || { echo "ERROR: Unity build not executable: $SIM_BIN" >&2; exit 1; }
  [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] || echo "WARNING: no DISPLAY set — Unity window may fail to open."
else
  SIM_WAIT=0
fi

T_BRIDGE=$SIM_WAIT
T_NODES=$((SIM_WAIT + 3))

if [ "$SIM" = "1" ]; then
  tmux_start_session "$SESSION" "sim" "cd '${WS_DIR}/vnav_builds/lab3' && ./lab3.x86_64"
  tmux_add_window "$SESSION" "bridge" "sleep ${T_BRIDGE}; ros2 launch tesse_ros_bridge tesse_quadrotor_bridge.launch.yaml"
else
  tmux_start_session "$SESSION" "bridge" "ros2 launch tesse_ros_bridge tesse_quadrotor_bridge.launch.yaml"
fi

tmux_add_window "$SESSION" "traj" "sleep ${T_NODES}; '${SCRIPTS_DIR}/run_traj.sh'"

if [ "$CONTROLLER" = "1" ]; then
  tmux_add_window "$SESSION" "controller" "sleep ${T_NODES}; '${SCRIPTS_DIR}/run_controller.sh'"
fi

if [ "$FOXGLOVE" = "1" ]; then
  tmux_add_window "$SESSION" "foxglove" "sleep ${T_NODES}; ros2 launch tesse_foxglove_pkg tesse_foxglove.launch.py"
fi

if [ "$RVIZ" = "1" ]; then
  tmux_add_window "$SESSION" "rviz" "sleep ${T_NODES}; rviz2 -d '${WS_DIR}/src/controller_pkg/rviz/lab3.rviz' --ros-args -p use_sim_time:=true"
fi

[ "$FOXGLOVE" = "1" ] && echo "Foxglove: connect to ws://localhost:8765 and import src/tesse_foxglove_pkg/config/tesse_foxglove.json"
tmux_attach "$SESSION" "$DETACH"
