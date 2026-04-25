#!/usr/bin/env bash
set -eo pipefail

LOG_DIR="$HOME/.ros/log"

if [ -d "$LOG_DIR" ]; then
  find "$LOG_DIR" -type f -name '*mavros*.log' | tail -n 5 | while read -r log_file; do
    echo "--- $log_file ---"
    tail -n 40 "$log_file" || true
  done
else
  echo "ROS_LOG_DIR_MISSING"
fi

