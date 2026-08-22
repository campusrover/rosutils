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

`ROS_DISTRO` must be exported *before* sourcing the role script — both role
scripts refuse to run without it.

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
- downloads Doppler secrets automatically **if** a token is already
  configured on that machine (`doppler configure get token`); otherwise
  prints `[doppler] no token configured — skipping secrets` and moves on —
  run `doppler login` once on that machine if you need secrets there
- sets up `mcfly` for shell-history search

### Mac (remote role)

The chain assumes a real Linux + ROS 2 install (`/opt/ros/$ROS_DISTRO/setup.bash`),
which macOS doesn't provide natively. In practice "Mac" means an Ubuntu VM
running on the Mac — set it up the same way *inside that VM*:

```bash
export ROS_DISTRO=<your ROS 2 distro>
source ~/rosutils/ros2_remote_bashrc.bash
```

Same dome-fleet vars and `mcfly` as above; no Doppler or platformio PATH step
(robot-only).

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
