#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="$HOME/PX4-Autopilot"

if [ ! -d "$PX4_DIR/.git" ]; then
  echo "PX4_MISSING=1"
  exit 1
fi

git -C "$PX4_DIR" fetch --all --tags
git -C "$PX4_DIR" submodule sync --recursive
git -C "$PX4_DIR" submodule update --init --recursive

echo "PX4_READY=$PX4_DIR"
du -sh "$PX4_DIR" || true

