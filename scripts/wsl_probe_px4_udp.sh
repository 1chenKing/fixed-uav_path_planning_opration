#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  pkill -f "sitl_multiple_run.sh -m plane -n 6 -w empty" || true
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x gazebo || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_probe_fw.log 2>&1 &
FW_PID=$!

sleep 25

python3 - <<'PY'
import socket
import time

ports = [14540, 14541, 14542]
duration = 8.0

for port in ports:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    deadline = time.time() + duration
    packets = 0
    sizes = []
    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(2048)
            packets += 1
            sizes.append(len(data))
            if packets <= 3:
                print(f"PORT {port} RX from {addr[0]}:{addr[1]} bytes={len(data)}")
        except socket.timeout:
            pass
    print(f"PORT {port} TOTAL_PACKETS={packets} SAMPLE_SIZES={sizes[:5]}")
    sock.close()
PY

echo "=== FW LOG TAIL ==="
tail -n 60 /tmp/codex_probe_fw.log || true

kill "$FW_PID" || true
cleanup
