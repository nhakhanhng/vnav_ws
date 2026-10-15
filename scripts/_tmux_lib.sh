#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# _tmux_lib.sh — Helpers to run each ROS node in its own tmux window.
# Sourced by run_tesse.sh / run_gazebo.sh. Not meant to be run directly.
# -----------------------------------------------------------------------------

# Build the shell command executed inside a tmux window:
#   source env -> run command -> keep shell open so errors stay readable.
_wrap_cmd() {
  local name="$1" cmd="$2"
  printf 'source %q || exit 1; echo "=== [%s] %s"; %s; ec=$?; echo; echo "=== [%s] exited with code $ec (shell kept open; Ctrl+D to close)"; exec bash' \
    "${SCRIPTS_DIR}/env.sh" "$name" "$cmd" "$cmd" "$name"
}

# tmux_start_session <session> <first-window-name> <command>
tmux_start_session() {
  local session="$1" name="$2" cmd="$3"
  command -v tmux >/dev/null || { echo "ERROR: tmux is not installed (sudo apt install tmux)" >&2; exit 1; }
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[tmux] Session '$session' already running — restarting it."
    "${SCRIPTS_DIR}/stop.sh" "$session" >/dev/null
  fi
  tmux new-session -d -s "$session" -n "$name" -c "$WS_DIR" "$(_wrap_cmd "$name" "$cmd")"
  tmux set-option -t "$session" mouse on >/dev/null
  tmux set-option -t "$session" remain-on-exit off >/dev/null
}

# tmux_add_window <session> <window-name> <command>
tmux_add_window() {
  local session="$1" name="$2" cmd="$3"
  tmux new-window -t "$session" -n "$name" -c "$WS_DIR" "$(_wrap_cmd "$name" "$cmd")"
}

# tmux_attach <session> <detach:0|1>
tmux_attach() {
  local session="$1" detach="$2"
  tmux select-window -t "${session}:0"
  echo
  echo "Session '$session' started. Windows:"
  tmux list-windows -t "$session" -F "  #{window_index}: #{window_name}"
  echo
  echo "  Switch window : Ctrl+b <number>   (or click — mouse is on)"
  echo "  Detach        : Ctrl+b d          (nodes keep running)"
  echo "  Re-attach     : tmux attach -t $session"
  echo "  Stop all      : scripts/stop.sh $session"
  echo
  if [ "$detach" = "1" ]; then return 0; fi
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$session"
  else
    tmux attach -t "$session"
  fi
}
