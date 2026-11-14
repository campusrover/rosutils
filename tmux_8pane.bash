#!/usr/bin/env bash
# Turn the current tmux window into the 8-pane layout
LAYOUT='4a1e,278x86,0,0{138x86,0,0[138x21,0,0,0,138x21,0,22,6,138x21,0,44,5,138x20,0,66,7],139x86,139,0[139x20,139,0,1,139x21,139,21,3,139x21,139,43,2,139x21,139,65,4]}'

n=$(tmux list-panes | wc -l)
while [ "$n" -lt 8 ]; do
  tmux split-window
  tmux select-layout tiled
  n=$((n + 1))
done
tmux select-layout "$LAYOUT"
