#!/usr/bin/env bash
set -eo pipefail

ps -ef | grep -E 'px4|gzserver|gzclient|gazebo|sitl_run' | grep -v grep || true

