#!/usr/bin/env bash
#
# multirepo.sh - jog-my-memory helper for dealing with all the individual
# git repos under ros2_ws/src.
#
# MINI SPEC / PROPOSAL
# ---------------------
# Problem: src/ contains ~19 separate git repos (dome, dome_control,
# linorobot2, etc). Keeping them all clean/pulled/pushed by hand is
# tedious and easy to forget one.
#
# Commands:
#   check   - for each repo, print branch name + `git status --short`
#             (nothing printed under a repo = clean). Read-only.
#   pull    - for each repo, `git pull` (fast-forward from GitHub).
#   push    - for each repo, `git push` (only pushes repos that have
#             commits ahead of the remote; repos with uncommitted
#             changes are skipped with a warning, not auto-committed).
#   help/-h - print usage.
#
# List of repos to operate on lives in repos.sh (REPOS array), kept
# separate from this script so it's easy to eyeball / edit without
# touching logic.
#
# Not hardened: no fancy error recovery, assumes repos.sh dirs exist
# under src/, assumes remotes are already configured per-repo.
# This script does NOT run `git add`/`git commit` for you - push just
# pushes whatever is already committed.
#
# Usage:
#   ./multirepo.sh check
#   ./multirepo.sh pull
#   ./multirepo.sh push
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$HOME/ros2_ws/src"

# shellcheck source=repos.sh
source "$SCRIPT_DIR/repos.sh"

usage() {
  grep '^#' "$0" | sed '1d;/^#!/d;s/^# \{0,1\}//'
}

for_each_repo() {
  local action="$1"
  local repo path
  for repo in "${REPOS[@]}"; do
    path="$SRC_DIR/$repo"
    if [ ! -d "$path/.git" ]; then
      echo "== $repo: SKIP (not found / not a git repo) =="
      continue
    fi
    "$action" "$repo" "$path"
  done
}

cmd_check() {
  local repo="$1" path="$2"
  local branch dirty
  branch="$(git -C "$path" rev-parse --abbrev-ref HEAD 2>/dev/null)"
  dirty="$(git -C "$path" status --short)"
  if [ -z "$dirty" ]; then
    echo "== $repo [$branch]: clean =="
  else
    echo "== $repo [$branch]: DIRTY =="
    echo "$dirty" | sed 's/^/    /'
  fi
}

cmd_pull() {
  local repo="$1" path="$2"
  echo "== $repo: pull =="
  git -C "$path" pull
}

cmd_push() {
  local repo="$1" path="$2"
  local dirty
  dirty="$(git -C "$path" status --short)"
  if [ -n "$dirty" ]; then
    echo "== $repo: SKIP (uncommitted changes, commit first) =="
    return
  fi
  echo "== $repo: push =="
  git -C "$path" push
}

main() {
  local sub="${1:-help}"
  case "$sub" in
    check) for_each_repo cmd_check ;;
    pull)  for_each_repo cmd_pull ;;
    push)  for_each_repo cmd_push ;;
    help|-h|--help) usage ;;
    *) echo "Unknown command: $sub"; usage; exit 1 ;;
  esac
}

main "$@"
