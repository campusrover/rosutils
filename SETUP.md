# Setting up rosutils on a new machine

rosutils provides a shared shell-env chain (roles, dome-fleet config, platform
detection, the `mr` alias) that every machine sources via its own local
`~/.bashrc`. `~/.bashrc` itself is per-machine and NOT part of this repo —
each new machine needs one small local block added by hand, shown below.

## 1. Clone rosutils

```bash
git clone git@github.com:campusrover/rosutils.git ~/rosutils
```

(Must be at `~/rosutils` — every script in this chain hardcodes that path.)

## 2. Add the role block to `~/.bashrc`

`ROS_DISTRO` must be exported *before* sourcing the role script, on any
platform that actually has a `/opt/ros/<distro>/setup.bash` to source
(macOS doesn't — see below, it's handled automatically).

### Raspberry Pi (robot role)

```bash
export ROS_DISTRO=<your ROS 2 distro, e.g. jazzy>
source ~/rosutils/ros2_robot_bashrc.bash
```

This also, via the chain it sources:
- sources `/opt/ros/$ROS_DISTRO/setup.bash`
- sets `DOME_MODE=robot`
- pulls in dome-fleet vars (`DOME_CAMERA`, `DOME_PIPER_*`, `DOME_VOICE_CONFIG`,
  `DOME_VISION_CONFIG`, `DOME_SPIN_SURVEY_MAP`) from `ros2_dome_bashrc.bash`
- adds `~/.platformio/penv/bin` to `PATH` (platformio/Teensy uploads)
- sets up `mcfly` for shell-history search

### Mac (remote role, native macOS)

Macs in this fleet run macOS directly, not Linux — most of this repo assumes
a real ROS 2 + Linux install, which a Mac doesn't have. `ros2_remote_bashrc.bash`
detects macOS (`uname -s = Darwin`) and skips the parts that don't apply there,
so it's safe to source without any of that installed:

```bash
source ~/rosutils/ros2_remote_bashrc.bash
```

(No need to set `ROS_DISTRO` on a Mac — it's only required on Linux, where
`/opt/ros/$ROS_DISTRO/setup.bash` actually exists.)

On a Mac this still gives you: `ROSUTILS_PLATFORM=macos`/`ROSUTILS_DIR`,
`DOME_MODE=remote`, the dome-fleet vars from `ros2_dome_bashrc.bash` (they're
just path/string exports, harmless even if those paths don't exist locally),
the `mr` alias, and all the general aliases in `ros2_common_alias.bash`. It
skips: sourcing ROS 2's `setup.bash`, and `mcfly` history search (only if
`mcfly` isn't installed — installs of it work fine).

What genuinely won't work on a Mac, because it needs a real ROS 2 install:
any ROS 2 command itself (`ros2 ...`, `colcon build`, the `cb`/`cw`/`cs`
aliases that assume a built `ros2_ws`), and anything under `mr` that expects
actual git repos checked out at `~/ros2_ws/src` (fine — `mr check` just
prints `SKIP (not found)` for repos that aren't there instead of erroring).

## 3. Verify

Open a new shell (or `sb` to re-source `~/.bashrc`), then:

```bash
echo $ROSUTILS_PLATFORM   # raspi | ubuntu-vm | macos | ubuntu — auto-detected, no setup needed
echo $DOME_MODE            # robot | remote
echo $DOME_CAMERA
mr check                   # lists git status for every repo in tools/mr/workspaces.sh
```

## Repos under `~/ros2_ws/src`

`mr` (see `tools/mr/`) expects the repos listed in `tools/mr/workspaces.sh`
to already exist under `~/ros2_ws/src`. On a fresh machine, clone the ones
you need by hand — `mr` doesn't auto-clone missing repos yet.

## Claude Code config

Run `mr update_claude` any time to sync the latest `.claude/` config from
`~/j3` into your repos (asks y/n per repo, replaces that repo's `.claude/`).
