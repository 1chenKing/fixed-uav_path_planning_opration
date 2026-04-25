#!/usr/bin/env bash
set -eo pipefail

LOG_DIR="$HOME/.ros/log"

if [ -d "$LOG_DIR" ]; then
  find "$LOG_DIR" -type f \( -name '*uav_1-mavros*.log' -o -name '*uav_2-mavros*.log' \) | tail -n 6 | while read -r log_file; do
    echo "--- $log_file ---"
    tail -n 30 "$log_file" || true
  done
fi

