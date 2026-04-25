#!/usr/bin/env bash
set -euo pipefail

WIN_SRC="/mnt/d/catkin_ws"
WSL_DST="$HOME/catkin_ws"

echo "WIN_SRC=$WIN_SRC"
echo "WSL_DST=$WSL_DST"

if [ ! -d "$WIN_SRC" ]; then
  echo "WIN_SOURCE_MISSING"
  exit 1
fi

if [ ! -d "$WSL_DST" ]; then
  echo "WSL_TARGET_MISSING"
  exit 1
fi

echo "WINDOWS_UI_FILE=$(realpath "$WIN_SRC/src/mission_ui/src/mission_ui/swarm_control_plugin.py")"
echo "WSL_UI_FILE=$(realpath "$WSL_DST/src/mission_ui/src/mission_ui/swarm_control_plugin.py")"

WIN_HASH=$(sha256sum "$WIN_SRC/src/mission_ui/src/mission_ui/swarm_control_plugin.py" | awk '{print $1}')
WSL_HASH=$(sha256sum "$WSL_DST/src/mission_ui/src/mission_ui/swarm_control_plugin.py" | awk '{print $1}')

echo "WIN_UI_SHA256=$WIN_HASH"
echo "WSL_UI_SHA256=$WSL_HASH"

if [ "$WIN_HASH" = "$WSL_HASH" ]; then
  echo "WORKSPACE_SYNC=OK"
else
  echo "WORKSPACE_SYNC=MISMATCH"
fi
