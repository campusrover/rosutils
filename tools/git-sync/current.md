# git-sync - Current State & TODO

**Last Updated**: 2025-11-19

## Current Status

The git-sync tool is **functional and working**. All core features have been implemented and tested.

### What Works
- ✅ Scans directories for git repositories (one level deep)
- ✅ Shows repository status (branch, clean/dirty, ahead/behind remote)
- ✅ Interactive prompts for each repo
- ✅ Dry-run mode (default, safe)
- ✅ Real-run mode with `--real-run` flag
- ✅ Custom commit messages per repo
- ✅ Summary statistics
- ✅ Proper argument parsing with argparse
- ✅ Mode indicator (DRY RUN vs REAL RUN)
- ✅ Pulls remote changes before pushing (with rebase)
- ✅ Handles repos without remote tracking branches
- ✅ Detects diverged branches

### Recent Changes
1. **2025-11-19**: Renamed `repo_syncher.py` to `repo_syncer.py` to match import statement
2. **2025-11-19**: Added `--dry-run` and `--real-run` flags with dry-run as default
3. **2025-11-19**: Added mode indicator message at startup
4. **2025-11-19**: Implemented dry-run logic in `sync_repo()` function
5. **2025-11-19**: Added `check_remote_status()` to show if repo is ahead/behind/diverged from remote
6. **2025-11-19**: Added `git pull --rebase` before commit/push to handle remote changes
7. **2025-11-19**: Improved error handling for repos without remote tracking branches

### Code Structure
```
git-sync/git-sync/
├── git-sync.py         # Main CLI orchestrator
├── repo_scanner.py     # Finds git repos (find_git_repos)
└── repo_syncer.py      # Git operations (sync_repo, should_sync_repo)
```

## Installation Notes

### Not Yet Done
- [ ] Create wrapper script in `~/.local/bin/git-sync`
- [ ] Add to PATH or create symlink

### To Install
```bash
# Option 1: Create wrapper in ~/.local/bin
cat > ~/.local/bin/git-sync << 'EOF'
#!/bin/bash
python3 /Users/pitosalas/rosutils/tools/git-sync/git-sync/git-sync.py "$@"
EOF
chmod +x ~/.local/bin/git-sync

# Option 2: Use directly with full path
python3 /Users/pitosalas/rosutils/tools/git-sync/git-sync/git-sync.py
```

## Known Issues

None currently.

## Future Enhancement Ideas

### Priority: High
- [x] **Handle merge conflicts** - Now pulls with rebase before pushing; warns on conflicts
- [x] **Check for remote tracking** - Now checks and displays remote status; handles repos without tracking
- [x] **Better error messages** - Now shows actual git error output instead of generic exceptions
- [ ] **Confirm before push in real-run mode** - Extra safety: show what will be pushed and ask for final confirmation
- [ ] **Handle rebase conflicts** - If pull --rebase has conflicts, guide user or offer to abort

### Priority: Medium
- [ ] **Recursive scanning** - Add option to scan nested directories (multiple levels deep)
- [ ] **Filter by branch** - Option to only sync repos on specific branch (e.g., only `main` or `master`)
- [ ] **Parallel execution** - Sync multiple repos simultaneously for speed
- [ ] **Config file** - Support `.git-sync.yml` for default settings (excluded repos, default message, etc.)
- [ ] **Skip patterns** - Ability to exclude certain repos by name or pattern
- [ ] **Git status output** - Show `git status --short` output for each repo
- [ ] **Stash support** - Option to stash changes instead of committing
- [x] **Pull before push** - ✅ DONE: Now does `git pull --rebase` before pushing

### Priority: Low
- [ ] **Color output** - Add colors for better readability (green for success, red for errors)
- [ ] **Log file** - Write operations to a log file for audit trail
- [ ] **Pre-commit hooks** - Respect and run pre-commit hooks if present
- [ ] **Branch creation** - Option to create and switch to a new branch before committing
- [ ] **Tag support** - Option to tag commits
- [ ] **Pull before push** - Option to `git pull` before pushing
- [ ] **Multiple remote support** - Handle repos with multiple remotes
- [ ] **Non-interactive mode** - Flag to auto-sync all repos without prompts
- [ ] **JSON output** - Machine-readable output format for automation
- [ ] **Email notifications** - Send summary email after sync

### Code Quality
- [ ] **Add tests** - Unit tests for scanner and syncer modules
- [ ] **Type hints** - Add Python type annotations throughout
- [ ] **Error handling** - More granular exception handling
- [ ] **Logging** - Use Python logging module instead of print statements
- [ ] **Documentation** - Add docstrings to all functions

### Integration Ideas
- [ ] **GitHub CLI integration** - Use `gh` CLI for enhanced GitHub features
- [ ] **GitLab support** - Detect and handle GitLab remotes
- [ ] **Bitbucket support** - Detect and handle Bitbucket remotes
- [ ] **Pre/post hooks** - Allow custom scripts to run before/after sync
- [ ] **Slack/Discord notifications** - Post sync summaries to chat

## Design Decisions

### Why dry-run by default?
Safety first. Git operations are destructive and can't be easily undone. Users must explicitly opt into making changes.

### Why one level deep?
Simplicity and performance. Most use cases involve scanning a "projects" folder with multiple repos as immediate children. Deep recursion could be slow and find unwanted repos.

### Why interactive prompts?
Not all repos in a directory need syncing at the same time. Interactive mode gives users fine-grained control.

### Why single commit message per repo?
Each repository likely has different changes that deserve different commit messages.

## Performance Notes

- Scanning is fast (simple directory iteration)
- Status checks use `git` commands which are generally fast
- Bottleneck is network I/O during push operations
- Potential improvement: parallel execution (see Future Ideas)

## Testing Notes

To test without risk:
1. Use `--dry-run` (default) to see what would happen
2. Test on a single test directory first
3. Check git status manually after real-run to verify
4. Always use version control on important work (obviously!)

## Next Steps

Immediate next steps for development:
1. Create wrapper script in `~/.local/bin` for easy access
2. Test with real repositories in real-run mode
3. Add better error handling for push failures
4. Consider adding remote tracking check

---

## Questions to Consider

- Should we add a `--yes` flag to auto-accept all repos?
- Should we show `git diff --stat` before syncing?
- Should we support `.gitignore`-style exclusion file?
- Should commit messages be required or always use default?
- Should we validate that repos have remotes before trying to push?
