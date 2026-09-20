#!/usr/bin/env bash
#
# Apply this board's managed-component patches to the fetched component tree.
#
# The component manager downloads dependencies into <target>/managed_components/,
# which is .gitignored and regenerated, so hand-edits there don't survive. This
# script re-applies the tracked *.patch files in this directory after the fetch.
# It is wired into the target's top-level CMakeLists.txt so it runs on every
# configure (after the component manager, before compile).
#
# Idempotent: a patch that already applies in reverse is treated as applied and
# skipped. Non-fatal: a patch that no longer matches (e.g. the component bumped)
# only warns — it never fails the build. Re-running is always safe.
set -u

TARGET_DIR="${1:?usage: apply.sh <target_dir>}"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

shopt -s nullglob
for patch in "$PATCH_DIR"/*.patch; do
    name="$(basename "$patch")"
    # A clean forward dry-run means the patch is NOT yet applied (the pristine
    # text it removes is still present) -> apply it. If the forward dry-run fails,
    # the patch is either already applied or no longer matches the fetched tree;
    # either way we leave the file alone. (A reverse dry-run is unreliable here —
    # BSD patch's fuzzy matching reports false positives on the unpatched file.)
    if patch -p1 -d "$TARGET_DIR" --dry-run --forward <"$patch" >/dev/null 2>&1; then
        if patch -p1 -d "$TARGET_DIR" --forward <"$patch" >/dev/null 2>&1; then
            echo "component-patches: applied ${name}"
        else
            echo "component-patches: WARNING could not apply ${name}"
        fi
    else
        echo "component-patches: ${name} already applied (or no longer matches the fetched tree)"
    fi
done
exit 0
