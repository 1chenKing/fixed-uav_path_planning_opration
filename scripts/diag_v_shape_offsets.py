#!/usr/bin/env python3
import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from mission_ui.swarm_control_plugin import compute_formation_positions


def projection(point, heading_deg):
    heading_rad = math.radians(heading_deg)
    forward = (math.cos(heading_rad), math.sin(heading_rad))
    left = (-forward[1], forward[0])
    return (
        point[0] * forward[0] + point[1] * forward[1],
        point[0] * left[0] + point[1] * left[1],
    )


def main():
    vehicle_ids = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5"]
    headings = [0.0, 90.0, 180.0, 270.0]
    ok = True
    for heading in headings:
        positions = compute_formation_positions("v_shape", 30.0, heading, (0.0, 0.0), vehicle_ids)
        leader_forward, leader_left = projection(positions["uav_1"], heading)
        followers = []
        for vehicle_id in vehicle_ids[1:]:
            f_value, l_value = projection(positions[vehicle_id], heading)
            followers.append((vehicle_id, f_value, l_value))
        leader_ahead = all(f_value < leader_forward - 20.0 for _vehicle_id, f_value, _l_value in followers)
        first_pair_symmetric = abs(followers[0][1] - followers[1][1]) < 1e-6 and abs(followers[0][2] + followers[1][2]) < 1e-6
        second_pair_symmetric = abs(followers[2][1] - followers[3][1]) < 1e-6 and abs(followers[2][2] + followers[3][2]) < 1e-6
        heading_ok = leader_ahead and first_pair_symmetric and second_pair_symmetric
        ok = ok and heading_ok
        print(
            "HEADING={:.0f} LEADER_AHEAD={} PAIR1_SYM={} PAIR2_SYM={}".format(
                heading,
                leader_ahead,
                first_pair_symmetric,
                second_pair_symmetric,
            ),
            flush=True,
        )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
