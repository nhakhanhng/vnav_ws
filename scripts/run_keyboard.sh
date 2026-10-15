#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_keyboard.sh — Gazebo keyboard helper: W = arm (hover speed), R = respawn, Q = quit.
#
# NOTE: lab3_keyboard is not registered as a console entry point in
# gazebo_quadrotor_pkg/setup.py, so it is started as a Python module here.
# WARNING: when armed it publishes on rotor_speed_cmds and FIGHTS controller_node.
#          Use it without the controller, or only for the R (respawn) key.
# -----------------------------------------------------------------------------
source "$(dirname "${BASH_SOURCE[0]}")/env.sh" || exit 1
exec python3 -m gazebo_quadrotor_pkg.lab3_keyboard --ros-args -p initial_z:="${SPAWN_Z:-2.0}"
