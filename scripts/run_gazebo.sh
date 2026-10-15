#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_gazebo.sh — Full Lab 3 stack on the Gazebo Classic substitute simulator,
# one tmux window per component:
#
#   0 gazebo      gzserver/gzclient + spawn + robot_state_publisher
#                 + quadrotor_bridge + lab3_viz_publisher (/visuals)
#   1 traj        traj_publisher        -> /desired_state
#   2 controller  controller_node       -> /rotor_speed_cmds
#   3 keyboard    (optional) W arm / R respawn / Q quit
#   4 foxglove    (optional) foxglove_bridge on ws://localhost:8765
#   5 rviz        (optional) rviz2
#
# Usage:
#   scripts/run_gazebo.sh [options]
#
# Options:
#   --headless         run Gazebo without GUI (gzserver only)
#   --paused           start the physics paused
#   --spawn-z Z        initial height of the quadrotor (default 2.0)
#   --foxglove         also start foxglove_bridge
#   --rviz             also start RViz2
#   --keyboard         also start the keyboard helper (conflicts with controller when armed!)
#   --no-controller    do not start controller_node
#   --lab4             load worlds/lab4.world and show trajectory vertices
#   --detach           start in background without attaching to tmux
#   -h, --help         show this help
#
# Stop everything:  scripts/stop.sh gazebo
# -----------------------------------------------------------------------------
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "${SCRIPTS_DIR}/_tmux_lib.sh"

SESSION="vnav_gazebo"
GUI=true; PAUSED=false; SPAWN_Z=2.0
FOXGLOVE=0; RVIZ=0; KEYBOARD=0; CONTROLLER=1; DETACH=0; LAB4=0

while [ $# -gt 0 ]; do
  case "$1" in
    --headless)      GUI=false ;;
    --paused)        PAUSED=true ;;
    --spawn-z)
      [ $# -ge 2 ] || { echo "ERROR: --spawn-z requires a number" >&2; exit 1; }
      SPAWN_Z="$2"; shift ;;
    --foxglove)      FOXGLOVE=1 ;;
    --rviz)          RVIZ=1 ;;
    --keyboard)      KEYBOARD=1 ;;
    --no-controller) CONTROLLER=0 ;;
    --lab4)          LAB4=1 ;;
    --detach)        DETACH=1 ;;
    -h|--help)       awk 'NR>1 && /^#/{sub(/^# ?/,"");print;next} NR>1{exit}' "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 1 ;;
  esac
  shift
done

[[ "$SPAWN_Z" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]] || {
  echo "ERROR: --spawn-z must be a finite number" >&2; exit 1; }

ros2 pkg prefix gazebo_quadrotor_pkg >/dev/null 2>&1 || {
  echo "ERROR: gazebo_quadrotor_pkg not built. Run: scripts/build.sh gazebo_quadrotor_pkg" >&2; exit 1; }
ros2 pkg prefix gazebo_ros >/dev/null 2>&1 || {
  echo "ERROR: gazebo_ros is unavailable. Install the Gazebo Classic ROS packages." >&2; exit 1; }
if [ "$FOXGLOVE" = "1" ]; then
  ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 || {
    echo "ERROR: foxglove_bridge is unavailable." >&2; exit 1; }
fi

LAUNCH_ARGS="gui:=${GUI} paused:=${PAUSED} spawn_z:=${SPAWN_Z}"
if [ "$LAB4" = "1" ]; then
  WORLD="$(ros2 pkg prefix gazebo_quadrotor_pkg)/share/gazebo_quadrotor_pkg/worlds/lab4.world"
  LAUNCH_ARGS="${LAUNCH_ARGS} world:=${WORLD} show_traj_vertices:=true show_reference_circle:=false"
fi

T_NODES=6   # seconds for Gazebo to come up and spawn the model

tmux_start_session "$SESSION" "gazebo" "ros2 launch gazebo_quadrotor_pkg gazebo_quadrotor.launch.py ${LAUNCH_ARGS}"
tmux_add_window "$SESSION" "traj" "sleep ${T_NODES}; '${SCRIPTS_DIR}/run_traj.sh'"

if [ "$CONTROLLER" = "1" ]; then
  tmux_add_window "$SESSION" "controller" "sleep ${T_NODES}; '${SCRIPTS_DIR}/run_controller.sh'"
fi

if [ "$KEYBOARD" = "1" ]; then
  [ "$CONTROLLER" = "1" ] && echo "WARNING: keyboard 'W' (arm) publishes rotor speeds that fight controller_node. Use 'R' to respawn only."
  tmux_add_window "$SESSION" "keyboard" "sleep ${T_NODES}; SPAWN_Z=${SPAWN_Z} '${SCRIPTS_DIR}/run_keyboard.sh'"
fi

if [ "$FOXGLOVE" = "1" ]; then
  tmux_add_window "$SESSION" "foxglove" "ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true"
  echo "Foxglove: connect to ws://localhost:8765 and import src/gazebo_quadrotor_pkg/config/lab3_foxglove.json"
fi

if [ "$RVIZ" = "1" ]; then
  tmux_add_window "$SESSION" "rviz" "sleep ${T_NODES}; rviz2 -d '${WS_DIR}/src/controller_pkg/rviz/lab3.rviz' --ros-args -p use_sim_time:=true"
fi

tmux_attach "$SESSION" "$DETACH"
