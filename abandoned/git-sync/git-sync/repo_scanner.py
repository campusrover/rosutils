#!/usr/bin/env python3
# repo_scanner: Find git repositories in directory tree
# Author: Pito Salas and Claude
# Open Source Under MIT License

from pathlib import Path


def find_git_repos(root_path):
    root = Path(root_path)
    repos = []
    
    for item in root.iterdir():
        if item.is_dir() and not item.name.startswith("."):
            if (item / ".git").exists():
                repos.append(item)
    
    return sorted(repos)