# Claude Code config

Portable Claude Code settings, kept here so a new machine can be set up the
same way.

## Files

- `statusline.sh` — the status line script. Shows model, current directory,
  and the 5-hour and weekly usage limits.
- `settings.json` — user-level Claude Code settings, including the
  `statusLine` entry that runs the script.

## What the status line shows

```
Opus | dome_vision | 5h: 40% (10/24/26 9:38; 10/24/26 12:08) | 1wk: 25% (...)
```

Each limit segment reads **percent used (reset time; projected run-out time)**.

- The 5h segment is green; the weekly segment is yellow.
- *Run-out* assumes the current rate of use holds for the rest of the window.
  If it falls after the reset time, you won't hit the limit.

## Install

```bash
cp ~/rosutils/claude/statusline.sh ~/.claude/
cp ~/rosutils/claude/settings.json ~/.claude/
```

**The second copy overwrites** any existing `~/.claude/settings.json`. On a
machine that already has its own settings, add only the `statusLine` block by
hand.

Restart Claude Code to pick it up.

## Requirements

- `jq` must be installed.
- Works on Linux and macOS. The script tries GNU `date -d @epoch` first and
  falls back to BSD `date -r epoch`.
