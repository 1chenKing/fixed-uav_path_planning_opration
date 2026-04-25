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
    rospy.init_node("diag_v_shape_transition", anonymous=True)
    app = QApplication.instance() or QApplication([])
    widget = SwarmControlWidget()

    widget._mission_phase = "mission_active"
    widget._safe_anchor_xy = (210.0, 140.0)
    widget._task_points = [
        {"x": 210.0, "y": 140.0, "z": 60.0},
        {"x": 260.0, "y": 210.0, "z": 60.0},
        {"x": 320.0, "y": 280.0, "z": 60.0},
    ]

    set_combo_by_data(widget._formation_type, "v_shape")
    widget._spacing.setValue(24.0)
    widget._heading.setValue(135.0)
    widget._anchor_x.setValue(210.0)
    widget._anchor_y.setValue(140.0)
    widget._anchor_z.setValue(60.0)

    # Simulate an in-flight cluster that has not yet opened into a V.
    current_positions = {
        "uav_1": (118.0, 96.0, 58.0),
        "uav_2": (122.0, 100.0, 58.0),
        "uav_3": (120.0, 104.0, 58.0),
        "uav_4": (124.0, 108.0, 58.0),
        "uav_5": (119.0, 112.0, 58.0),
        "uav_6": (123.0, 116.0, 58.0),
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
    print("REGROUP_POINTS", widget._regroup_points, flush=True)

    first_targets = {}
    for vehicle_id in VEHICLE_IDS:
        mission_items = push_clients[vehicle_id].calls[-1][1]
        first_index = 1 if mission_items and mission_items[0].command == 22 and len(mission_items) > 1 else 0
        print("{} FIRST_CMD {}".format(vehicle_id, mission_items[0].command), flush=True)
        first_targets[vehicle_id] = decode_waypoint_to_local(
            fixes[vehicle_id],
            current_positions[vehicle_id],
            mission_items[first_index],
        )

    print("FIRST_TARGETS", flush=True)
    for vehicle_id in VEHICLE_IDS:
        x_value, y_value, z_value = first_targets[vehicle_id]
        print("{} {:.2f} {:.2f} {:.1f}".format(vehicle_id, x_value, y_value, z_value), flush=True)

    app.quit()


if __name__ == "__main__":
    main()
