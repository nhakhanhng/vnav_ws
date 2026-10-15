#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_controller.sh — Run the geometric controller_node.
#
# Reads gains from src/controller_pkg/config/params.yaml (the SOURCE file, so
# gain edits take effect without rebuilding). Gains can be overridden inline:
#
#   scripts/run_controller.sh
#   scripts/run_controller.sh kx=10 kv=6 kr=120 komega=20
#   PARAMS=/path/to/other.yaml scripts/run_controller.sh
# -----------------------------------------------------------------------------
source "$(dirname "${BASH_SOURCE[0]}")/env.sh" || exit 1

PARAMS="${PARAMS:-${WS_DIR}/src/controller_pkg/config/params.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"

OVERRIDES=()
for kv in "$@"; do
  case "$kv" in
    *=*) OVERRIDES+=(-p "${kv%%=*}:=${kv#*=}") ;;
    *)   echo "ERROR: invalid override '$kv' (expected name=value)" >&2; exit 1 ;;
  esac
done

echo "[controller] params: $PARAMS  use_sim_time: $USE_SIM_TIME  overrides: ${OVERRIDES[*]:-none}"
exec ros2 run controller_pkg controller_node --ros-args \
  --params-file "$PARAMS" \
  -p use_sim_time:="$USE_SIM_TIME" \
  "${OVERRIDES[@]}"
