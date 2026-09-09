#!/usr/bin/env bash
# Build the distributable mod using only production sources and assets.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
version="${1:-}"
if [[ ! "$version" =~ ^v[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9]+([.-][A-Za-z0-9]+)*)?$ ]]; then
    echo 'Usage: ./package.sh vMAJOR.MINOR.PATCH[-prerelease] [build.sh options]' >&2
    exit 2
fi
shift
"$repo_root/build.sh" "$@" --configuration Release
python3 - "$repo_root" "$version" <<'PY'
import hashlib
from pathlib import Path
import sys
import zipfile

root = Path(sys.argv[1])
version = sys.argv[2]
assets = root / 'Assets/PyLoN'
mod = root / 'GameData/PyLoN'
# Enumerate production inputs, so caches, symbols and reference DLLs cannot leak.
relative_paths = sorted(p.relative_to(assets) for p in assets.rglob('*') if p.is_file())
for relative in relative_paths:
    if relative.suffix not in {'.cfg', '.mu', '.png'}:
        raise SystemExit(f'Unexpected asset type: {relative}')
    if (mod / relative).read_bytes() != (assets / relative).read_bytes():
        raise SystemExit(f'Generated asset differs from source: {relative}')
relative_paths.extend([Path('Plugins/PyLoN.dll'), Path('LICENSE')])
if (mod / 'LICENSE').read_bytes() != (root / 'LICENSE').read_bytes():
    raise SystemExit('Generated license differs from source: LICENSE')
for relative in relative_paths:
    if not (mod / relative).is_file() or (mod / relative).is_symlink():
        raise SystemExit(f'Missing or symlinked release file: {relative}')

output = root / 'dist'
output.mkdir(exist_ok=True)
archive = output / f'PyLoN-{version}.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as package:
    for relative in relative_paths:
        package.write(mod / relative, (Path('GameData/PyLoN') / relative).as_posix())
checksum = hashlib.sha256(archive.read_bytes()).hexdigest()
archive.with_suffix('.zip.sha256').write_text(f'{checksum}  {archive.name}\n', encoding='utf-8')
print(f'Release archive: {archive}')
PY
