#!/usr/bin/env bash
# Build and install PyLoN. Demos are opt-in; no Development dependency.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ksp_install="${KSPDIR:-$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program}"
ros_workspace="${ROS2_WS:-$HOME/ros2_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
lock_file="${PYLON_SYNC_LOCK_FILE:-${TMPDIR:-/tmp}/pylon-sync.lock}"
skip_ksp_build=0; skip_ksp_sync=0; skip_ros2_sync=0; skip_ros2_build=0
demos=()
usage() {
    cat <<'USAGE'
Usage: ./sync.sh [options]
  --demo NAME         Include debris_orbit, position_estimator or mun_rover (repeatable).
  --all-demos         Include all three demos and optional perception dependencies.
  --skip-ksp-build    Skip plugin build.
  --skip-ksp-sync     Skip KSP installation.
  --skip-ros2-sync    Skip ROS source synchronization.
  --skip-ros2-build   Skip colcon build.
  -h, --help          Show help.
Environment: KSPDIR, ROS2_WS, ROS_SETUP, PYLON_SYNC_LOCK_FILE.
Core packages: pylon_interfaces, pylon_bridge, pylon_vehicle_control.
Recognized pre-PyLoN installations are backed up outside GameData/src before replacement.
USAGE
}
while (($#)); do
    case "$1" in
        --demo)
            [[ $# -ge 2 ]] || { usage >&2; exit 2; }
            case "$2" in
                debris_orbit|position_estimator|mun_rover) demos+=("$2");;
                *) echo "Unknown demo: $2" >&2; exit 2;;
            esac
            shift 2;;
        --all-demos) demos+=(debris_orbit position_estimator mun_rover); shift;;
        --skip-ksp-build) skip_ksp_build=1; shift;;
        --skip-ksp-sync) skip_ksp_sync=1; shift;;
        --skip-ros2-sync) skip_ros2_sync=1; shift;;
        --skip-ros2-build) skip_ros2_build=1; shift;;
        -h|--help) usage; exit 0;;
        *) echo "Unknown argument: $1" >&2; exit 2;;
    esac
done
command -v flock >/dev/null
exec 9>"$lock_file"
flock 9
trap 'flock -u 9' EXIT
packages=(pylon_interfaces pylon_bridge pylon_vehicle_control)
source_dirs=(Ros2/pylon_interfaces Ros2/pylon_bridge Ros2/pylon_vehicle_control)
for demo in "${demos[@]}"; do
    package="pylon_demo_$demo"
    if [[ " ${packages[*]} " != *" $package "* ]]; then
        packages+=("$package"); source_dirs+=("Demo/$package")
    fi
    if [[ "$demo" != debris_orbit && " ${packages[*]} " != *" pylon_perception "* ]]; then
        packages+=(pylon_perception); source_dirs+=(Ros2/pylon_perception)
    fi
done
sync_dir() {
    [[ -d "$1" ]] || { echo "Missing source: $1" >&2; exit 1; }
    mkdir -p "$2"
    rsync -a --delete --exclude __pycache__ --exclude '*.pyc' --exclude .pytest_cache "$1/" "$2/"
}
if (( !skip_ksp_build )); then "$repo_root/build.sh" --ksp-dir "$ksp_install"; fi
if (( !skip_ksp_sync )); then
    [[ -d "$ksp_install/GameData" ]] || { echo "Missing KSP GameData: $ksp_install" >&2; exit 1; }
    python3 "$repo_root/Migration/pylon_migrate.py" --retire-install --apply --ksp-dir "$ksp_install"
    target="$ksp_install/GameData/PyLoN"
    mkdir -p "$target/Config"
    # Runtime.cfg is user configuration: seed once, then preserve on upgrades.
    if [[ ! -f "$target/Config/Runtime.cfg" ]]; then cp "$repo_root/GameData/PyLoN/Config/Runtime.cfg" "$target/Config/Runtime.cfg"; fi
    rsync -a --delete --exclude Config/Runtime.cfg "$repo_root/GameData/PyLoN/" "$target/"
fi
if (( !skip_ros2_sync )); then
    python3 "$repo_root/Migration/pylon_migrate.py" --retire-install --apply --ros2-ws "$ros_workspace"
    for i in "${!packages[@]}"; do sync_dir "$repo_root/${source_dirs[$i]}" "$ros_workspace/src/${packages[$i]}"; done
fi
if (( !skip_ros2_build )); then
    [[ -f "$ros_setup" ]] || { echo "Missing ROS setup: $ros_setup" >&2; exit 1; }
    set +u
    source "$ros_setup"
    set -u
    [[ "${ROS_DISTRO:-}" == jazzy ]] || { echo 'ROS 2 Jazzy is required' >&2; exit 1; }
    cd "$ros_workspace"
    colcon build --packages-up-to "${packages[@]}"
fi
printf 'PyLoN sync complete. Packages: %s\n' "${packages[*]}"
