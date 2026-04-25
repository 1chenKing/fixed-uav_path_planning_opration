#!/usr/bin/env bash
set -euo pipefail

echo "=== /tmp/single_fw_diag_mavros.log ==="
tail -n 120 /tmp/single_fw_diag_mavros.log || true
echo "=== /tmp/single_fw_diag_px4.log ==="
tail -n 120 /tmp/single_fw_diag_px4.log || true
