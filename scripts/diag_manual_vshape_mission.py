#!/usr/bin/env python3
import math
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rospy
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import NavSatFix

from mission_ui.swarm_control_plugin import SwarmControlWidget, VEHICLE_IDS


class FakeMissionClear:
    def __call__(self):
        return SimpleNamespace(success=True)


class FakeMissionPush:
    def __init__(self):
        self.calls = []

    def __call__(self, start_index, mission_items):
        self.calls.append((start_index, mission_items))
        return SimpleNamespace(success=True, wp_transfered=len(mission_items))


def set_combo_by_data(combo, value):
    for index in range(combo.count()):
        if combo.itemData(index) == value:
            combo.setCurrentIndex(index)
            return
    raise RuntimeError("combo value not found: {}".format(value))


def decode_waypoint_to_local(fix, local_pose, waypoint):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(fix.latitude)), 1.0)
    east_m = (waypoint.y_long - fix.longitude) * lon_scale
    north_m = (waypoint.x_lat - fix.latitude) * lat_scale
    return (local_pose[0] + east_m, local_pose[1] + north_m, waypoint.z_alt)


def main():
    rospy.init_node("diag_manual_vshape_mission", anonymous=True)
    app = QApplication.instance() or QApplication([])
    widget = SwarmControlWidget()

    set_combo_by_data(widget._formation_type, "v_shape")
    widget._spacing.setValue(35.0)
    widget._heading.setValue(90.0)
    widget._anchor_x.setValue(13.0)
    widget._anchor_y.setValue(-12.0)
    widget._anchor_z.setValue(80.0)
    widget._task_points = [{"x": 13.0, "y": -12.0, "z": 80.0}]
    widget._anchor_xy = (13.0, -12.0)
    widget._safe_anchor_xy = (13.0, -12.0)
    widget._mission_phase = "formation_ready"
    widget._scenario_key = "manual"

    current_positions = {
        "uav_1": (-25.0, -5.0, 78.0),
        "uav_2": (-35.0, -12.0, 78.0),
        "uav_3": (-45.0, -19.0, 78.0),
        "uav_4": (-55.0, -26.0, 78.0),
        "uav_5": (-65.0, -33.0, 78.0),
        "uav_6": (-75.0, -40.0, 78.0),
    }
    widget._vehicle_positions = current_positions.copy()

    fixes = {}
    push_clients = {}
    clear_clients = {}
    for vehicle_id in VEHICLE_IDS:
        fix = NavSatFix()
        fix.latitude = 47.3977419
        fix.longitude = 8.5455938
        fix.altitude = 535.0
        fixes[vehicle_id] = fix
        clear_clients[vehicle_id] = FakeMissionClear()
        push_clients[vehicle_id] = FakeMissionPush()
    widget._global_positions = fixes
    widget._waypoint_clear_clients = clear_clients
    widget._waypoint_push_clients = push_clients

    widget._upload_formation_mission_worker()

    print("ROUTE_MODE", widget._route_mode, flush=True)
    print("STAGING", widget._launch_staging_points, flush=True)
    print("PLANNED", widget._planned_task_path[:8], flush=True)
    print("MISSION_COUNTS", widget._mission_counts, flush=True)
    for vehicle_id in VEHICLE_IDS:
        calls = push_clients[vehicle_id].calls
        if not calls:
            print(vehicle_id, "NO_MISSION", flush=True)
            continue
        mission_items = calls[-1][1]
        print(vehicle_id, "MISSION_LEN", len(mission_items), flush=True)
        for index, waypoint in enumerate(mission_items[:6]):
            local_point = decode_waypoint_to_local(fixes[vehicle_id], current_positions[vehicle_id], waypoint)
            print(
                "{} WP{} CMD{} {:.2f} {:.2f} {:.1f}".format(
                    vehicle_id,
                    index,
                    waypoint.command,
                    local_point[0],
                    local_point[1],
                    local_point[2],
                ),
                flush=True,
            )

    app.quit()


if __name__ == "__main__":
    main()
