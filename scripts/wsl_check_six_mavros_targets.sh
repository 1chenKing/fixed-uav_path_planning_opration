#!/usr/bin/env bash
set -eo pipefail

LOG_DIR="$HOME/.ros/log"

if [ -d "$LOG_DIR" ]; then
  find "$LOG_DIR" -type f -name '*mavros*.log' | tail -n 12 | while read -r log_file; do
    echo "--- $log_file ---"
    grep -E 'FCU URL|TARGET ID|Remote address' "$log_file" || true
  done
fi

