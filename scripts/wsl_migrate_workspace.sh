#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="/mnt/d/catkin_ws"
DST_DIR="$HOME/catkin_ws"

mkdir -p "$DST_DIR"
cp -a "$SRC_DIR"/. "$DST_DIR"/

echo "MIGRATED_TO=$DST_DIR"
ls -la "$DST_DIR"

