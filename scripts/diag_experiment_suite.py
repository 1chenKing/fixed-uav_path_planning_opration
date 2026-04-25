#!/usr/bin/env python3
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_SRC = Path(__file__).resolve().parents[1] / "src" / "mission_ui" / "src"
if str(REPO_SRC) not in sys.path:
    sys.path.insert(0, str(REPO_SRC))

import rospy
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import NavSatFix

from mission_ui.experiment_support import PLANNER_MODE_OPTIONS, scenario_options
from mission_ui.swarm_control_plugin import SwarmControlWidget, VEHICLE_IDS, compute_formation_positions


class FakePublisher:
    def __init__(self, *_args, **_kwargs):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeSubscriber:
    def __init__(self, *_args, **_kwargs):
        pass

    def unregister(self):
        return None


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


def build_global_fixes():
    fixes = {}
    for vehicle_id in VEHICLE_IDS:
        fix = NavSatFix()
        fix.latitude = 47.3977419
        fix.longitude = 8.5455938
        fix.altitude = 535.0
        fixes[vehicle_id] = fix
    return fixes


def build_fake_mission_clients():
    clear_clients = {}
    push_clients = {}
    for vehicle_id in VEHICLE_IDS:
        clear_clients[vehicle_id] = FakeMissionClear()
        push_clients[vehicle_id] = FakeMissionPush()
    return clear_clients, push_clients


def seed_vehicle_positions(widget, scenario_key):
    formation_type = widget._formation_type.currentData()
    spacing = float(widget._spacing.value())
    heading_deg = float(widget._heading.value())
    anchor_x = float(widget._anchor_x.value())
    anchor_y = float(widget._anchor_y.value())
    anchor_z = float(widget._anchor_z.value())

    if scenario_key == "dynamic_reconfiguration":
        widget._vehicle_positions = {
            "uav_1": (118.0, 96.0, 58.0),
            "uav_2": (122.0, 100.0, 58.0),
            "uav_3": (120.0, 104.0, 58.0),
            "uav_4": (124.0, 108.0, 58.0),
            "uav_5": (119.0, 112.0, 58.0),
            "uav_6": (123.0, 116.0, 58.0),
        }
        widget._mission_phase = "mission_active"
        return

    expected = compute_formation_positions(
        formation_type,
        spacing,
        heading_deg,
        (anchor_x, anchor_y),
        VEHICLE_IDS,
    )
    heading_rad = math.radians(heading_deg)
    offset_x = math.cos(heading_rad) * 85.0
    offset_y = math.sin(heading_rad) * 85.0
    widget._vehicle_positions = {
        vehicle_id: (xy[0] - offset_x, xy[1] - offset_y, anchor_z - 2.0)
        for vehicle_id, xy in expected.items()
    }
    widget._mission_phase = "armed"


def main():
    rospy.Publisher = lambda *args, **kwargs: FakePublisher(*args, **kwargs)
    rospy.Subscriber = lambda *args, **kwargs: FakeSubscriber(*args, **kwargs)
    rospy.wait_for_service = lambda *args, **kwargs: None
    rospy.get_param = lambda _name, default=None: default
    rospy.Time.now = staticmethod(lambda: rospy.Time())
    rospy.sleep = lambda duration: time.sleep(float(duration))
    rospy.logwarn = lambda *args, **kwargs: None
    app = QApplication.instance() or QApplication([])
    widget = SwarmControlWidget()
    widget.hide()

    widget._global_positions = build_global_fixes()

    workspace_root = Path(__file__).resolve().parents[1]
    results_dir = workspace_root / "results" / "swarm_experiments"
    results_dir.mkdir(parents=True, exist_ok=True)

    suite_rows = []
    suite_snapshots = []

    for _scenario_label, scenario_key in scenario_options():
        set_combo_by_data(widget._scenario_selector, scenario_key)
        widget._apply_selected_scenario()
        for planner_label, planner_mode in PLANNER_MODE_OPTIONS:
            set_combo_by_data(widget._planner_mode_combo, planner_mode)
            widget._on_planner_mode_changed()
            seed_vehicle_positions(widget, scenario_key)
            widget._safe_anchor_xy = (float(widget._anchor_x.value()), float(widget._anchor_y.value()))
            widget._vehicle_last_seen = {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}
            widget._waypoint_clear_clients, widget._waypoint_push_clients = build_fake_mission_clients()
            widget._upload_formation_mission_worker()
            widget._refresh_live_metrics()

            snapshot = widget._collect_experiment_snapshot()
            snapshot["planner_label"] = planner_label
            suite_snapshots.append(snapshot)

            suite_rows.append(
                {
                    "scenario": snapshot["scenario_label"],
                    "planner_mode": snapshot["planner_mode"],
                    "planner_label": planner_label,
                    "route_mode": snapshot["route_mode"],
                    "task_success_rate": snapshot.get("task_success_rate", 0.0),
                    "avg_route_length_m": snapshot.get("avg_route_length_m", 0.0),
                    "avg_energy_wh": snapshot.get("avg_energy_wh", 0.0),
                    "min_clearance_m": snapshot.get("min_clearance_m", 0.0),
                    "avg_formation_error_m": snapshot.get("avg_formation_error_m", 0.0),
                    "formation_keep_score": snapshot.get("formation_keep_score", 0.0),
                    "mission_refresh_count": snapshot.get("mission_refresh_count", 0),
                }
            )

            image_name = "{}_{}_suite.png".format(scenario_key, planner_mode)
            widget.grab().save(str(results_dir / image_name))

    csv_path = results_dir / "suite_summary.csv"
    json_path = results_dir / "suite_summary.json"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(suite_rows[0].keys()))
        writer.writeheader()
        writer.writerows(suite_rows)
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(suite_snapshots, handle, ensure_ascii=False, indent=2)

    print("SUITE_ROWS", len(suite_rows), flush=True)
    print("CSV", csv_path, flush=True)
    print("JSON", json_path, flush=True)
    app.quit()


if __name__ == "__main__":
    main()
