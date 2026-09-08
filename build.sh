#!/usr/bin/env bash
set -euo pipefail

configuration="Release"
ksp_dir="${KSPDIR:-}"

usage() {
    cat <<'USAGE'
Usage: ./build.sh [options]

KSP modをビルドします。KSPインストール先への同期は行いません。

Options:
  --ksp-dir PATH        KSPインストール先（KSPDIRでも指定可能）
  --configuration NAME ビルド構成（既定: Release）
  -h, --help           ヘルプを表示
USAGE
}

require_value() {
    if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then
        echo "Option requires a value: $1" >&2
        exit 2
    fi
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --ksp-dir)
            require_value "$@"
            ksp_dir="$2"
            shift 2
            ;;
        --configuration)
            require_value "$@"
            configuration="$2"
            shift 2
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

if [ -z "$ksp_dir" ]; then
    ksp_dir="$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program"
fi

managed="$ksp_dir/KSP_x64_Data/Managed"
if [ ! -f "$managed/Assembly-CSharp.dll" ]; then
    managed="$ksp_dir/KSP_Data/Managed"
fi
if [ ! -f "$managed/Assembly-CSharp.dll" ]; then
    echo "Could not find KSP managed assemblies under: $ksp_dir" >&2
    exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dotnet="$script_dir/.dotnet-linux-net8/dotnet"
if [ ! -x "$dotnet" ]; then
    dotnet="dotnet"
fi

export DOTNET_CLI_HOME="$script_dir/.dotnet_home"
export NUGET_PACKAGES="$script_dir/.nuget/packages"
export DOTNET_SKIP_FIRST_TIME_EXPERIENCE=1
export DOTNET_NOLOGO=1

project_path="$script_dir/Source/KerbalLiDAR/KerbalLiDAR.csproj"
assets_path="$script_dir/Source/KerbalLiDAR/obj/project.assets.json"
restore_args=()
if [[ -f "$assets_path" && ! "$project_path" -nt "$assets_path" && ! "$script_dir/NuGet.Config" -nt "$assets_path" ]]; then
    restore_args+=(--no-restore)
    echo "Using cached NuGet assets: $assets_path"
fi

"$dotnet" build "$project_path" \
    -c "$configuration" \
    -p:KSPDIR="$ksp_dir" \
    -p:KSPManagedDir="$managed" \
    "${restore_args[@]}" \
    --configfile "$script_dir/NuGet.Config"

echo "KerbalLiDAR mod folder is ready at: $script_dir/GameData/KerbalLiDAR"
