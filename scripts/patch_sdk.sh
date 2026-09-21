#!/usr/bin/env bash
# Applies the workshop's patches to the pinned SOSLAB_SDK submodule.
# Idempotent: an already-patched working copy is left alone.
set -euo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="$LAB_ROOT/SOSLAB_SDK"
PATCH_DIR="$LAB_ROOT/patches"

if [ ! -f "$SDK_DIR/CMakeLists.txt" ]; then
  echo "SOSLAB_SDK submodule is missing; clone with --recurse-submodules" >&2
  exit 1
fi

shopt -s nullglob
patches=("$PATCH_DIR"/*.patch)
if [ "${#patches[@]}" -eq 0 ]; then
  echo "no patches to apply"
  exit 0
fi

# The patches are a dependent series: each one is cut against the tree the
# previous one produced, so a later patch rewrites the context an earlier one
# matched on. That makes a mid-series patch impossible to check on its own once
# the series is applied. The last patch settles the state of the whole series.
last="${patches[${#patches[@]} - 1]}"
if git -C "$SDK_DIR" apply --reverse --check "$last" >/dev/null 2>&1; then
  echo "${#patches[@]} patches already applied"
  exit 0
fi

for patch in "${patches[@]}"; do
  name="$(basename "$patch")"
  if git -C "$SDK_DIR" apply "$patch"; then
    echo "$name applied"
  else
    echo "$name failed to apply; the submodule commit may have moved." >&2
    echo "The SDK working copy may now be partly patched. Reset it with:" >&2
    echo "  git submodule update --force --checkout SOSLAB_SDK" >&2
    exit 1
  fi
done
