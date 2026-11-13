#!/usr/bin/env bash
# Mill 0.11 has no output-directory option. Give each configuration its own
# working directory, retaining source/dependency links and a private out/.
set -euo pipefail
xs_root=$(cd "$(dirname "$0")/.." && pwd)
xs_workspace=${XS_MILL_WORKSPACE:?Set XS_MILL_WORKSPACE to an absolute build workspace}
# TopMain also writes ElaborationArtefacts to ./build after emitting RTL.
mkdir -p "$xs_workspace/build"
exec 9>"$xs_workspace/.mill.lock"
flock 9
for input in src rocket-chip huancun fudian difftest; do
    if [[ ! -e "$xs_workspace/$input" ]]; then
        ln -s "$xs_root/$input" "$xs_workspace/$input"
    fi
done
cp "$xs_root/build.sc" "$xs_workspace/build.sc"
cp "$xs_root/.mill-version" "$xs_workspace/.mill-version"
cd "$xs_workspace"
exec mill -i --home "$xs_workspace/.ammonite" "$@"
