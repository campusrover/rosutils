#!/usr/bin/env python3
# repo_syncer: Handle git commit and push operations
# Author: Pito Salas and Claude
# Open Source Under MIT License

import subprocess as sp
from pathlib import Path


def should_sync_repo(repo_path):
    print(f"\n📁 {repo_path.name}")
    print_repo_status(repo_path)
    response = input("Sync this repo? (y/n): ").strip().lower()
    return response == "y"


def print_repo_status(repo_path):
    try:
        branch = get_current_branch(repo_path)
        changes = has_uncommitted_changes(repo_path)
        remote_status = check_remote_status(repo_path)
        print(f"   Branch: {branch}")
        if changes:
            print(f"   Status: Has uncommitted changes")
        else:
            print(f"   Status: Clean")
        if remote_status:
            print(f"   Remote: {remote_status}")
    except sp.CalledProcessError:
        print(f"   Status: Unable to determine (may not be valid repo)")


def get_current_branch(repo_path):
    result = sp.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        cwd=repo_path,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def has_uncommitted_changes(repo_path):
    result = sp.run(
        ["git", "status", "--porcelain"],
        cwd=repo_path,
        capture_output=True,
        text=True,
        check=True,
    )
    return bool(result.stdout.strip())


def check_remote_status(repo_path):
    """Check if local branch is behind, ahead, or diverged from remote."""
    try:
        # Fetch remote info without downloading
        sp.run(
            ["git", "fetch", "--dry-run"],
            cwd=repo_path,
            capture_output=True,
            text=True,
            check=True,
        )

        # Check relationship to upstream
        result = sp.run(
            ["git", "rev-list", "--left-right", "--count", "HEAD...@{upstream}"],
            cwd=repo_path,
            capture_output=True,
            text=True,
            check=True,
        )

        ahead, behind = result.stdout.strip().split()
        ahead, behind = int(ahead), int(behind)

        if ahead > 0 and behind > 0:
            return f"Diverged (ahead {ahead}, behind {behind})"
        elif behind > 0:
            return f"Behind remote by {behind} commit(s)"
        elif ahead > 0:
            return f"Ahead of remote by {ahead} commit(s)"
        else:
            return "Up to date with remote"
    except sp.CalledProcessError:
        # No upstream or other error
        return None


def sync_repo(repo_path, dry_run=True):
    msg = get_commit_message()

    if dry_run:
        print("   [DRY RUN] Would execute:")
        print("      git pull --rebase")
        print("      git add -A")
        print(f"      git commit -m \"{msg}\"")
        print("      git push")
        print("   ✓ Would sync successfully")
        return True

    try:
        # Pull with rebase to get remote changes first
        print("   Pulling remote changes...")
        result = sp.run(
            ["git", "pull", "--rebase"],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )

        if result.returncode != 0:
            # Pull failed - might be due to no remote or conflicts
            if "no tracking information" in result.stderr or "does not have any commits yet" in result.stderr:
                print("   ⚠ No remote tracking branch, skipping pull")
            else:
                print(f"   ✗ Pull failed: {result.stderr.strip()}")
                return False

        # Stage all changes
        sp.run(
            ["git", "add", "-A"],
            cwd=repo_path,
            check=True,
            capture_output=True,
        )

        # Commit (might fail if nothing to commit after pull)
        result = sp.run(
            ["git", "commit", "-m", msg],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )

        if result.returncode != 0:
            if "nothing to commit" in result.stdout:
                print("   ℹ Nothing to commit (working tree clean)")
            else:
                print(f"   ✗ Commit failed: {result.stderr.strip()}")
                return False

        # Push changes
        result = sp.run(
            ["git", "push"],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )

        if result.returncode != 0:
            print(f"   ✗ Push failed: {result.stderr.strip()}")
            return False

        print("   ✓ Synced successfully")
        return True
    except sp.CalledProcessError as e:
        stderr_msg = e.stderr.decode() if hasattr(e.stderr, 'decode') else str(e.stderr) if e.stderr else str(e)
        print(f"   ✗ Sync failed: {stderr_msg}")
        return False


def get_commit_message():
    default = "synchronize work"
    prompt = f'Commit message [{default}]: '
    response = input(prompt).strip()
    return response if response else default