#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# stop.sh — Gracefully stop a running stack (Ctrl+C every window, then kill).
#
#   scripts/stop.sh            # stop both stacks
#   scripts/stop.sh tesse      # stop TESSE stack   (session vnav_tesse)
#   scripts/stop.sh gazebo     # stop Gazebo stack  (session vnav_gazebo)
# -----------------------------------------------------------------------------
target="${1:-all}"
case "$target" in
  tesse|vnav_tesse)   SESSIONS=(vnav_tesse) ;;
  gazebo|vnav_gazebo) SESSIONS=(vnav_gazebo) ;;
  all)                SESSIONS=(vnav_tesse vnav_gazebo) ;;
  *) echo "Usage: $0 [tesse|gazebo|all]" >&2; exit 1 ;;
esac

for s in "${SESSIONS[@]}"; do
  if tmux has-session -t "$s" 2>/dev/null; then
    echo "[stop] Sending Ctrl+C to all windows of '$s'..."
    for w in $(tmux list-windows -t "$s" -F '#{window_index}'); do
      tmux send-keys -t "${s}:${w}" C-c 2>/dev/null || true
    done
    sleep 3
    tmux kill-session -t "$s" 2>/dev/null || true
    echo "[stop] '$s' stopped."
  else
    echo "[stop] '$s' is not running."
  fi

done
exit 0
