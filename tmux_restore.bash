#!/bin/bash
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

SESSION=main

if ! tmux has-session -t $SESSION 2>/dev/null; then
  tmux new-session -d -s $SESSION

  # build 2x2 grid
  tmux split-window -h -t $SESSION          # right column
  tmux split-window -v -t $SESSION:0.0      # split left column
  tmux split-window -v -t $SESSION:0.2      # split right column

  # send commands to panes (skips empty ones)
  [ -n "$PANE0" ] && tmux send-keys -t $SESSION:0.0 "$PANE0" Enter
  [ -n "$PANE1" ] && tmux send-keys -t $SESSION:0.1 "$PANE1" Enter
  [ -n "$PANE2" ] && tmux send-keys -t $SESSION:0.2 "$PANE2" Enter
  [ -n "$PANE3" ] && tmux send-keys -t $SESSION:0.3 "$PANE3" Enter
fi

exec tmux -CC attach -t $SESSION