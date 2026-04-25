#!/usr/bin/env bash
set -euo pipefail

cd /home/chen/catkin_ws/code

ls -la
printf '\n===KEY===\n'
find . -maxdepth 2 \( -name README.md -o -name README -o -name package.xml -o -name CMakeLists.txt \) | sort
printf '\n===FILES===\n'
find . -maxdepth 3 -type f | sed -n '1,240p'
