#!/usr/bin/env bash
set -euo pipefail

configuration="Release"
ksp_dir="${KSPDIR:-}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --ksp-dir)
            ksp_dir="$2"
            shift 2
            ;;
        --configuration)
            configuration="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
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

"$dotnet" build "$script_dir/Source/KerbalLiDAR/KerbalLiDAR.csproj" \
    -c "$configuration" \
    -p:KSPDIR="$ksp_dir" \
    -p:KSPManagedDir="$managed" \
    --configfile "$script_dir/NuGet.Config"

echo "KerbalLiDAR mod folder is ready at: $script_dir/GameData/KerbalLiDAR"
