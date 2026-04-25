#!/usr/bin/env python3
import math
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rospy
from geometry_msgs.msg import Pose, Vector3
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import NavSatFix
from swarm_msgs.msg import Obstacle

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


def make_obstacle(obstacle_id, x_value, y_value, sx_value, sy_value):
    obstacle = Obstacle()
    obstacle.id = obstacle_id
    obstacle.shape = "box"
    obstacle.pose = Pose()
    obstacle.pose.position.x = x_value
    obstacle.pose.position.y = y_value
    obstacle.pose.position.z = 0.0
    obstacle.size = Vector3(sx_value, sy_value, 140.0)
    obstacle.enabled = True
    return obstacle


def configure_common_widget(widget, formation_type):
    set_combo_by_data(widget._formation_type, formation_type)
    widget._spacing.setValue(35.0)
    widget._heading.setValue(45.0)
    widget._anchor_x.setValue(520.0)
    widget._anchor_y.setValue(320.0)
    widget._anchor_z.setValue(80.0)
    widget._task_points = [{"x": 520.0, "y": 320.0, "z": 80.0}]
    widget._anchor_xy = (520.0, 320.0)
    widget._safe_anchor_xy = (520.0, 320.0)
    widget._mission_phase = "formation_ready"
    widget._scenario_key = "manual"
    widget._planner_mode = "adaptive"
    widget._vehicle_positions = {
        "uav_1": (-20.0, -8.0, 78.0),
        "uav_2": (-32.0, -12.0, 78.0),
        "uav_3": (-44.0, -16.0, 78.0),
        "uav_4": (-56.0, -20.0, 78.0),
        "uav_5": (-68.0, -24.0, 78.0),
        "uav_6": (-80.0, -28.0, 78.0),
    }
    widget._obstacles = {
        "diag_obstacle_01": make_obstacle("diag_obstacle_01", 210.0, 130.0, 125.0, 125.0),
        "diag_obstacle_02": make_obstacle("diag_obstacle_02", 315.0, 205.0, 110.0, 120.0),
    }
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


def main():
    rospy.init_node("diag_staging_alignment", anonymous=True)
    app = QApplication.instance() or QApplication([])
    formations = ["line", "v_shape", "echelon_left", "echelon_right", "column"]
    for formation_type in formations:
        widget = SwarmControlWidget()
        configure_common_widget(widget, formation_type)
        widget._upload_formation_mission_worker()
        margin = widget._center_route_margin_for_scenario(formation_type, float(widget._spacing.value()))
        staging_count = len(widget._launch_staging_points)
        staging_path_count = len(widget._launch_staging_path)
        staging_collision = widget._route_has_collisions(widget._launch_staging_path, margin * 0.85)
        display_path = widget._display_planned_path()
        duplicate_start = False
        if staging_count >= 3 and len(display_path) >= 6:
            first_stage = widget._launch_staging_points[0]
            duplicate_start = sum(
                1
                for point in display_path[:8]
                if math.hypot(point["x"] - first_stage["x"], point["y"] - first_stage["y"]) < 5.0
            ) > 1
        print(
            "{} STAGING={} STAGE_PATH={} COLLISION={} DUP_START={} PLAN_POINTS={}".format(
                formation_type,
                staging_count,
                staging_path_count,
                staging_collision,
                duplicate_start,
                len(widget._planned_task_path),
            ),
            flush=True,
        )
        app.processEvents()
    app.quit()


if __name__ == "__main__":
    main()
