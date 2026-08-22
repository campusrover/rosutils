#!/usr/bin/env python3
# git_sync: Scan subdirectories for git repos and sync them to GitHub
# Author: Pito Salas and Claude
# Open Source Under MIT License

import argparse
import os
import sys
from pathlib import Path

import repo_scanner as scanner
import repo_syncer as syncer


def main(start_path, dry_run=True):
    mode = "DRY RUN" if dry_run else "REAL RUN"
    print(f"=== Running in {mode} mode ===\n")

    repos = scanner.find_git_repos(start_path)

    if not repos:
        print(f"No git repositories found in {start_path}")
        return

    print(f"Found {len(repos)} git repository(ies)\n")

    synced = 0
    failed = 0

    for repo_path in repos:
        if syncer.should_sync_repo(repo_path):
            result = syncer.sync_repo(repo_path, dry_run=dry_run)
            if result:
                synced += 1
            else:
                failed += 1

    print(f"\n--- Summary ---")
    print(f"Synced: {synced}")
    print(f"Failed: {failed}")
    print(f"Skipped: {len(repos) - synced - failed}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Scan subdirectories for git repos and sync them to GitHub"
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=os.getcwd(),
        help="Directory path to scan for git repositories (default: current directory)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=True,
        help="Show what would be done without making changes (default)"
    )
    parser.add_argument(
        "--real-run",
        action="store_true",
        help="Actually perform git operations"
    )

    args = parser.parse_args()

    # --real-run overrides the default --dry-run
    dry_run = not args.real_run

    main(args.path, dry_run=dry_run)