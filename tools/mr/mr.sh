#!/usr/bin/env bash
#
# mr - jog-my-memory helper for dealing with the individual git repos
# spread across named workspaces (e.g. ros2_ws/src). Grows over time to
# hold other small cross-repo/cross-machine chores, not just repo sync.
#
# MINI SPEC / PROPOSAL
# ---------------------
# Problem: ros2_ws/src contains ~15 separate git repos (dome, dome_control,
# linorobot2, etc). Keeping them all clean/pulled/pushed by hand is
# tedious and easy to forget one. Other repo groups (workspaces) may be
# added later (e.g. a ros_vision workspace).
#
# Commands (all take an optional workspace name, default: ros2_ws):
#   check   [ws] - for each repo, print branch name + `git status --short`
#             (nothing printed under a repo = clean). Read-only.
#   pull    [ws] - for each repo, `git pull` (fast-forward from GitHub).
#   push    [ws] - for each repo, `git push` (only pushes repos that have
#             commits ahead of the remote; repos with uncommitted
#             changes are skipped with a warning, not auto-committed).
#   update_claude [ws] - `git pull` in ~/j3, then for each repo ask y/n
#             and, on yes, replace the repo's .claude/ with a copy of
#             ~/j3/.claude. Does not add/commit/push in the repos.
#   help/-h - print usage.
#
# Workspaces (name -> source dir + repo list) live in workspaces.sh,
# kept separate from this script so it's easy to eyeball / edit without
# touching logic.
#
# Not hardened: no fancy error recovery, assumes workspaces.sh dirs
# exist, assumes remotes are already configured per-repo. This script
# does NOT run `git add`/`git commit` for you - push just pushes
# whatever is already committed.
#
# Usage (once the `mr` alias is set up, see ros2_common_alias.bash):
#   mr check
#   mr pull
#   mr push
#   mr update_claude
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=workspaces.sh
source "$SCRIPT_DIR/workspaces.sh"

usage() {
  grep '^#' "$0" | sed '1d;/^#!/d;s/^# \{0,1\}//'
}

resolve_workspace() {
  local ws="${1:-$DEFAULT_WORKSPACE}"
  if [ -z "${WORKSPACE_DIR[$ws]+set}" ]; then
    echo "Unknown workspace: $ws (known: ${!WORKSPACE_DIR[*]})" >&2
    exit 1
  fi
  echo "$ws"
}

for_each_repo() {
  local action="$1" ws="$2"
  local src_dir="${WORKSPACE_DIR[$ws]}"
  local repos_var="REPOS_${ws}[@]"
  local repo path

  for repo in "${!repos_var}"; do
    path="$src_dir/$repo"
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

cmd_update_claude() {
  local repo="$1" path="$2"
  local reply=""
  read -r -p "Copy .claude into $repo? [y/N] " reply </dev/tty || true
  case "$reply" in
    y|Y|yes|YES)
      rm -rf "$path/.claude"
      cp -r "$J3_CLAUDE_DIR" "$path/.claude"
      echo "== $repo: .claude updated =="
      ;;
    *)
      echo "== $repo: skipped =="
      ;;
  esac
}

cmd_update_claude_all() {
  local ws="$1"
  local j3_dir="$HOME/j3"
  J3_CLAUDE_DIR="$j3_dir/.claude"

  if [ ! -d "$j3_dir/.git" ]; then
    echo "$j3_dir is not a git repo, aborting" >&2
    exit 1
  fi
  echo "== j3: pull =="
  git -C "$j3_dir" pull

  if [ ! -d "$J3_CLAUDE_DIR" ]; then
    echo "$J3_CLAUDE_DIR not found, aborting" >&2
    exit 1
  fi

  for_each_repo cmd_update_claude "$ws"
}

main() {
  local sub="${1:-help}"
  local ws
  case "$sub" in
    check|pull|push|update_claude)
      ws="$(resolve_workspace "${2:-}")" || exit 1
      ;;
  esac
  case "$sub" in
    check)  for_each_repo cmd_check "$ws" ;;
    pull)   for_each_repo cmd_pull "$ws" ;;
    push)   for_each_repo cmd_push "$ws" ;;
    update_claude) cmd_update_claude_all "$ws" ;;
    help|-h|--help) usage ;;
    *) echo "Unknown command: $sub"; usage; exit 1 ;;
  esac
}

main "$@"
