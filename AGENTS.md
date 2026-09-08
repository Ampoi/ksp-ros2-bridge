# Agent Instructions

After making any code, config, asset, or documentation change that affects the KSP mod or ROS2 bridge, run:

```bash
./dev_sync.sh
```

This script rebuilds the KSP plugin, syncs `GameData/KerbalLiDAR` into the local KSP install, syncs `Ros2/ksp_lidar_bridge` into `~/ros2_ws/src`, and runs `colcon build --packages-select ksp_lidar_bridge`.

Use these environment variables when the defaults are wrong:

```bash
KSPDIR="/path/to/Kerbal Space Program" ROS2_WS="$HOME/ros2_ws" ./dev_sync.sh
```

Default paths:

- `KSPDIR=$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program`
- `ROS2_WS=$HOME/ros2_ws`
- `ROS_SETUP=/opt/ros/jazzy/setup.bash`

If only one side changed, scoped runs are acceptable:

```bash
./dev_sync.sh --skip-ros2-sync --skip-ros2-build
./dev_sync.sh --skip-ksp-build --skip-ksp-sync
```

Before reporting completion, mention whether `./dev_sync.sh` succeeded. If it cannot run because external paths require approval or are missing, state that clearly and include the exact command that should be run.

The tracked production entrypoint is `./sync.sh`. `./dev_sync.sh` is an
optional, ignored local compatibility command; when it is absent in a clean
clone, run `./sync.sh` with the same scope flags instead. Production builds
must not include or depend on files under `Development/` or legacy `Tools/`.
