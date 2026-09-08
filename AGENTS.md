# Agent Instructions

After making any code, config, asset, or documentation change that affects the KSP mod or ROS2 bridge, run:

```bash
./dev sync
```

This script rebuilds the KSP plugin, syncs `GameData/PyLoN` into the local KSP install, syncs `Ros2/pylon_bridge` into `~/ros2_ws/src`, and runs `colcon build --packages-select pylon_bridge`.

Use these environment variables when the defaults are wrong:

```bash
KSPDIR="/path/to/Kerbal Space Program" ROS2_WS="$HOME/ros2_ws" ./dev sync
```

Default paths:

- `KSPDIR=$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program`
- `ROS2_WS=$HOME/ros2_ws`
- `ROS_SETUP=/opt/ros/jazzy/setup.bash`

If only one side changed, scoped runs are acceptable:

```bash
./dev sync --skip-ros2-sync --skip-ros2-build
./dev sync --skip-ksp-build --skip-ksp-sync
```

Before reporting completion, mention whether `./dev sync` succeeded. If it cannot run because external paths require approval or are missing, state that clearly and include the exact command that should be run.

The tracked production entrypoint is `./sync.sh`. `./dev` is an
optional, ignored local development entrypoint backed by `Development/commands/`;
when it is absent in a clean clone, run `./sync.sh` with the same scope flags instead. Production builds
must not include or depend on files under `Development/` or legacy `Tools/`.

Before investigative debugging or work on KSP implementation details, read
`Development/AGENTS.md` when that local file exists. Keep probes, experimental
code, evidence, and authoring sources under ignored `Development/`; production
changes must remain independent of that directory. Local-only storage is not
permission to reverse engineer third-party software.
