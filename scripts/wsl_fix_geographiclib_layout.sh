#!/usr/bin/env bash
set -eo pipefail

mkdir -p /usr/share/GeographicLib/geoids
mkdir -p /usr/share/GeographicLib/gravity
mkdir -p /usr/share/GeographicLib/magnetic

for candidate in \
  /home/chen/catkin_ws/GeographicLib/geoids/egm96-5.pgm \
  /home/chen/catkin_ws/GeographicLib/gravity/egm96-5.pgm \
  /home/chen/catkin_ws/GeographicLib/egm96-5.pgm; do
  if [ -f "$candidate" ]; then
    cp -f "$candidate" /usr/share/GeographicLib/geoids/egm96-5.pgm
    echo "COPIED_GEOID_FROM=$candidate"
    exit 0
  fi
done

echo "GEOID_SOURCE_NOT_FOUND"
exit 1

