#!/usr/bin/env bash
set -euo pipefail

PX4_DIR=/home/chen/catkin_ws/PX4_Firmware

echo "=== AIRSPEED SELECTOR ==="
grep -R "Airspeed selector module down" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null || true
grep -R "SENS_EN_ARSPDSIM" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null | sed -n '1,40p' || true
echo "--- airspeedCheck.cpp ---"
sed -n '1,240p' "$PX4_DIR/src/modules/commander/HealthAndArmingChecks/checks/airspeedCheck.cpp" || true

echo "=== EKF2 MISSING DATA ==="
grep -R "ekf2 missing data" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null || true
echo "--- estimatorCheck.cpp ---"
sed -n '1,220p' "$PX4_DIR/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp" || true

echo "=== SYSTEM POWER UNAVAILABLE ==="
grep -R "system power unavailable" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null || true
grep -R "SUPPLY_CHK\\|power unavailable" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null | sed -n '1,80p' || true

echo "=== GCS CONNECTION ==="
grep -R "No connection to the ground control station" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null || true
grep -R "ground control station" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null | sed -n '1,80p' || true
echo "--- rcAndDataLinkCheck.cpp ---"
sed -n '1,220p' "$PX4_DIR/src/modules/commander/HealthAndArmingChecks/checks/rcAndDataLinkCheck.cpp" || true

echo "=== LANDING WAYPOINT REQUIRED ==="
grep -R "Landing waypoint/pattern required" -n "$PX4_DIR/src" "$PX4_DIR/ROMFS" 2>/dev/null || true
