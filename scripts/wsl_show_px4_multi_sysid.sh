#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "=== sitl_multiple_run excerpts ==="
grep -nE "SYSID_THISMAV|MAV_SYS_ID|mavlink.*145|1458|1456|instance" \
  "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh" || true

for rootfs_dir in "$PX4_DIR"/build/px4_sitl_default/rootfs/*; do
  if [ -f "$rootfs_dir/etc/init.d-posix/rcS" ]; then
    echo "=== rcS excerpts: $rootfs_dir ==="
    grep -nE "SYSID_THISMAV|mavlink start|1458|1456" "$rootfs_dir/etc/init.d-posix/rcS" || true
  fi
done
