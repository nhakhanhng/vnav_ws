#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# build.sh — Build the workspace with colcon.
#
#   scripts/build.sh                         # build everything
#   scripts/build.sh controller_pkg          # build selected package(s)
#   scripts/build.sh --clean controller_pkg  # wipe build/install of package(s) first
# -----------------------------------------------------------------------------
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$WS_DIR"

CLEAN=0
if [ "${1:-}" = "--clean" ]; then CLEAN=1; shift; fi
PKGS=("$@")

if [ "$CLEAN" = "1" ]; then
  if [ ${#PKGS[@]} -eq 0 ]; then
    echo "[build] Removing build/ install/ log/"
    rm -rf build install log
  else
    for p in "${PKGS[@]}"; do
      echo "[build] Removing build/$p install/$p"
      rm -rf "build/$p" "install/$p"
    done
  fi
fi

ARGS=(--cmake-args -DCMAKE_BUILD_TYPE=Release)
if [ ${#PKGS[@]} -gt 0 ]; then
  colcon build --packages-select "${PKGS[@]}" "${ARGS[@]}"
else
  colcon build "${ARGS[@]}"
fi

echo
echo "[build] Done. Open a new shell or run:  source scripts/env.sh"
