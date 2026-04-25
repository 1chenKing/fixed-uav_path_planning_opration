#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="$HOME/PX4-Autopilot"

if [ -d "$PX4_DIR/.git" ]; then
  echo "PX4_ALREADY_EXISTS=$PX4_DIR"
  exit 0
fi

git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$PX4_DIR"
echo "PX4_CLONED_TO=$PX4_DIR"

