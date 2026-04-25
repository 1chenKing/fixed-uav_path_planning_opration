#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="$HOME/PX4-Autopilot"

if [ -d "$PX4_DIR/.git" ]; then
  echo "PX4_EXISTS=1"
  du -sh "$PX4_DIR" || true
  git -C "$PX4_DIR" rev-parse --is-inside-work-tree || true
else
  echo "PX4_EXISTS=0"
fi

