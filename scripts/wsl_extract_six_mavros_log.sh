#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/codex_six_ros.log}"

if [ ! -f "$LOG_FILE" ]; then
  echo "LOG_MISSING=$LOG_FILE"
  exit 0
fi

grep -E "FCU URL|TARGET ID|Remote address|MAVROS started|HEARTBEAT|udp0|connected" "$LOG_FILE" | tail -n 120 || true
