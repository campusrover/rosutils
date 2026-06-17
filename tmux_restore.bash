t#!/bin/bash
# rosutils/tmux_restore.bash
# Run on the robot: sets up tmux layout and attaches via iTerm2 tmux integration

# ── always pull latest rosutils first ─────────────────
cd ~/rosutils && git pull --quiet

# ── pane commands (leave empty for plain shell) ────────
PANE0=""   # top-left
PANE1=""   # bottom-left
PANE2=""   # top-right
PANE3=""   # bottom-right
# ───────────────────────────────────────────────────────

if [ -n "$TMUX" ]; then
  # already inside tmux (CC session) — split current window into 2x2
  echo "Already inside tmux, splitting current window..."
  tmux split-window -h
  tmux split-window -v -t 0
  tmux split-window -v -t 2

  [ -n "$PANE0" ] && tmux send-keys -t 0 "$PANE0" Enter
  [ -n "$PANE1" ] && tmux send-keys -t 1 "$PANE1" Enter
  [ -n "$PANE2" ] && tmux send-keys -t 2 "$PANE2" Enter
  [ -n "$PANE3" ] && tmux send-keys -t 3 "$PANE3" Enter
else
  SESSION=main
  if ! tmux has-session -t $SESSION 2>/dev/null; then
    tmux new-session -d -s $SESSION

    tmux split-window -h -t $SESSION
    tmux split-window -v -t $SESSION:0.0
    tmux split-window -v -t $SESSION:0.2

    [ -n "$PANE0" ] && tmux send-keys -t $SESSION:0.0 "$PANE0" Enter
    [ -n "$PANE1" ] && tmux send-keys -t $SESSION:0.1 "$PANE1" Enter
    [ -n "$PANE2" ] && tmux send-keys -t $SESSION:0.2 "$PANE2" Enter
    [ -n "$PANE3" ] && tmux send-keys -t $SESSION:0.3 "$PANE3" Enter
  fi
  exec tmux -CC attach -t $SESSION
fi