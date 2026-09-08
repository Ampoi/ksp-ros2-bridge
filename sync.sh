#!/usr/bin/env bash
# Build and install the production KSP mod and ROS2 packages from a clean clone.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ksp_dir="${KSPDIR:-$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program}"
ros2_ws="${ROS2_WS:-$HOME/ros2_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
sync_lock_file="${DEV_SYNC_LOCK_FILE:-${TMPDIR:-/tmp}/kerbal-lidar-dev-sync.lock}"

usage() {
    cat <<USAGE
Usage: ./sync.sh [options]

Build and sync KerbalLiDAR into KSP GameData, then sync and build the ROS2 packages.

Options:
  --skip-ksp-build    Do not rebuild the KSP plugin DLL before syncing.
  --skip-ksp-sync     Do not copy GameData/KerbalLiDAR into KSP.
  --skip-ros2-sync    Do not copy ROS2 packages into ROS2_WS/src.
  --skip-ros2-build   Do not run colcon build for the ROS2 packages.
  -h, --help          Show this help.

Environment:
  KSPDIR      KSP install directory. Default: $HOME/.local/share/Steam/steamapps/common/Kerbal Space Program
  ROS2_WS     ROS2 workspace. Default: $HOME/ros2_ws
  ROS_SETUP   ROS2 Jazzy setup script. Default: /opt/ros/jazzy/setup.bash
  DEV_SYNC_LOCK_FILE  Cross-worktree lock file. Default: ${TMPDIR:-/tmp}/kerbal-lidar-dev-sync.lock
USAGE
}

skip_ksp_build=0
skip_ksp_sync=0
skip_ros2_sync=0
skip_ros2_build=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-ksp-build)
            skip_ksp_build=1
            shift
            ;;
        --skip-ksp-sync)
            skip_ksp_sync=1
            shift
            ;;
        --skip-ros2-sync)
            skip_ros2_sync=1
            shift
            ;;
        --skip-ros2-build)
            skip_ros2_build=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

require_dir() {
    local path="$1"
    local label="$2"
    if [[ ! -d "$path" ]]; then
        echo "$label directory does not exist: $path" >&2
        exit 1
    fi
}

sync_dir() {
    local source_dir="$1"
    local target_dir="$2"
    require_dir "$source_dir" "Source"
    mkdir -p "$(dirname "$target_dir")"
    if ! command -v rsync >/dev/null 2>&1; then
        echo "rsync is required for clean syncs. Install rsync or copy manually." >&2
        exit 1
    fi
    rsync -a --delete "$source_dir/" "$target_dir/"
}

acquire_sync_lock() {
    if ! command -v flock >/dev/null 2>&1; then
        echo "flock is required to prevent concurrent worktree syncs." >&2
        exit 1
    fi

    mkdir -p "$(dirname "$sync_lock_file")"
    exec 9>"$sync_lock_file"
    if ! flock -n 9; then
        echo "Another sync.sh is running; waiting for lock: $sync_lock_file"
        flock 9
    fi
}

acquire_sync_lock
# Compiler build servers may inherit fd 9 and outlive this script. Explicitly
# unlock the shared file description on exit rather than relying on fd closure.
trap 'flock -u 9' EXIT

if [[ "$skip_ksp_build" -eq 0 ]]; then
    "$repo_root/build.sh" --ksp-dir "$ksp_dir"
fi

if [[ "$skip_ksp_sync" -eq 0 ]]; then
    require_dir "$ksp_dir/GameData" "KSP GameData"
    sync_dir "$repo_root/GameData/KerbalLiDAR" "$ksp_dir/GameData/KerbalLiDAR"
    echo "Synced KSP GameData to: $ksp_dir/GameData/KerbalLiDAR"
fi

if [[ "$skip_ros2_sync" -eq 0 ]]; then
    mkdir -p "$ros2_ws/src"
    sync_dir "$repo_root/Ros2/ksp_ros2_interfaces" "$ros2_ws/src/ksp_ros2_interfaces"
    sync_dir "$repo_root/Ros2/ksp_lidar_bridge" "$ros2_ws/src/ksp_lidar_bridge"
    sync_dir "$repo_root/Ros2/ksp_vehicle_control" "$ros2_ws/src/ksp_vehicle_control"
    sync_dir "$repo_root/Ros2/ksp_nav2_bringup" "$ros2_ws/src/ksp_nav2_bringup"
    sync_dir "$repo_root/Demo/debris_orbit" "$ros2_ws/src/debris_orbit"
    sync_dir "$repo_root/Demo/position_estimator" "$ros2_ws/src/position_estimator"
    echo "Synced ROS2 interface package to: $ros2_ws/src/ksp_ros2_interfaces"
    echo "Synced ROS2 bridge package to: $ros2_ws/src/ksp_lidar_bridge"
    echo "Synced ROS2 vehicle control package to: $ros2_ws/src/ksp_vehicle_control"
    echo "Synced ROS2 Nav2 package to: $ros2_ws/src/ksp_nav2_bringup"
    echo "Synced debris orbit demo to: $ros2_ws/src/debris_orbit"
    echo "Synced position estimator demo to: $ros2_ws/src/position_estimator"
fi

if [[ "$skip_ros2_build" -eq 0 ]]; then
    if [[ -f "$ros_setup" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "$ros_setup"
        set -u
    else
        echo "ROS2 setup script does not exist: $ros_setup" >&2
        echo "Install ROS2 Jazzy or set ROS_SETUP to its setup.bash." >&2
        exit 1
    fi
    if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
        echo "ROS2 Jazzy is required, but ROS_DISTRO is '${ROS_DISTRO:-unset}'." >&2
        echo "Set ROS_SETUP to the setup.bash for a ROS2 Jazzy installation." >&2
        exit 1
    fi
    cd "$ros2_ws"
    colcon build --packages-up-to ksp_lidar_bridge ksp_vehicle_control ksp_nav2_bringup debris_orbit position_estimator
fi

echo "Done."
