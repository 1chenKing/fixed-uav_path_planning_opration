#!/usr/bin/env bash
set -eo pipefail

echo "--- custom GeographicLib files ---"
if [ -d /home/chen/catkin_ws/GeographicLib ]; then
  find /home/chen/catkin_ws/GeographicLib -maxdepth 4 -type f
else
  echo "CUSTOM_DIR_MISSING"
fi

echo "--- system GeographicLib files ---"
if [ -d /usr/share/GeographicLib ]; then
  find /usr/share/GeographicLib -maxdepth 3 -type f | grep -E 'egm96|geoid|gravity|magnetic' || true
else
  echo "SYSTEM_DIR_MISSING"
fi

