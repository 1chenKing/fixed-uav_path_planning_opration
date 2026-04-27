#!/usr/bin/env python3
import csv
import heapq
import json
import math
import random
import threading
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint, ParamValue
from mavros_msgs.srv import CommandBool, CommandLong, ParamSet, SetMode, WaypointClear, WaypointPush
from std_msgs.msg import String
from python_qt_binding.QtCore import QPointF, QRectF, Qt, Signal
from python_qt_binding.QtGui import QColor, QBrush, QFont, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from rqt_gui_py.plugin import Plugin

from swarm_msgs.msg import FormationCommand, Obstacle, SwarmStatus
from mission_ui.experiment_support import (
    PLANNER_MODE_OPTIONS,
    estimate_route_energy_wh,
    get_scenario_preset,
    min_clearance_to_obstacles,
    planner_mode_label,
    polyline_length,
    scenario_options,
    workspace_root_from_file,
)


VEHICLE_IDS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]
VEHICLE_COLORS = ["#2ec4b6", "#ff9f1c", "#e71d36", "#3a86ff", "#8338ec", "#ff006e"]
DEFAULT_FIXED_WING_BODY_RADIUS_M = 8.0
DEFAULT_FIXED_WING_TRACKING_MARGIN_M = 10.0
DEFAULT_FIXED_WING_TURN_BUFFER_M = 8.0
FORMATION_OPTIONS = [
    ("横队", "line"),
    ("V 字队形", "v_shape"),
    ("左梯队", "echelon_left"),
    ("右梯队", "echelon_right"),
    ("纵队", "column"),
]
SHAPE_OPTIONS = [("方盒", "box"), ("圆柱", "cylinder")]


def formation_vehicle_ids(formation_type, vehicle_ids=None):
    ids = list(vehicle_ids or VEHICLE_IDS)
    if formation_type == "v_shape" and len(ids) > 5:
        return ids[:5]
    return ids


def standby_vehicle_ids(formation_type, vehicle_ids=None):
    ids = list(vehicle_ids or VEHICLE_IDS)
    active_ids = formation_vehicle_ids(formation_type, ids)
    return [vehicle_id for vehicle_id in ids if vehicle_id not in active_ids]


def build_anchor_pose(x_value, y_value, z_value):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x_value
    pose.pose.position.y = y_value
    pose.pose.position.z = z_value
    pose.pose.orientation.w = 1.0
    return pose


def compute_formation_positions(formation_type, spacing, heading_deg, anchor_xy, vehicle_ids):
    offsets = compute_formation_offsets(formation_type, spacing, vehicle_ids)
    heading_rad = math.radians(heading_deg)
    forward = (math.cos(heading_rad), math.sin(heading_rad))
    left = (-forward[1], forward[0])
    anchor_x, anchor_y = anchor_xy
    positions = {}
    for vehicle_id, (forward_offset, left_offset) in zip(vehicle_ids, offsets):
        x_value = anchor_x + forward[0] * forward_offset + left[0] * left_offset
        y_value = anchor_y + forward[1] * forward_offset + left[1] * left_offset
        positions[vehicle_id] = (x_value, y_value)
    return positions


def compute_formation_offsets(formation_type, spacing, vehicle_ids):
    offsets = []
    for index, _vehicle_id in enumerate(vehicle_ids):
        if formation_type == "column":
            offset = (-spacing * index, 0.0)
        elif formation_type == "v_shape":
            pair_step = (index + 1) // 2
            side = -1.0 if index % 2 else 1.0
            offset = (
                (0.0, 0.0)
                if index == 0
                else (-spacing * pair_step * 1.05, side * spacing * pair_step * 0.95)
            )
        elif formation_type == "echelon_left":
            offset = (-spacing * index, spacing * index)
        elif formation_type == "echelon_right":
            offset = (-spacing * index, -spacing * index)
        else:
            offset = (0.0, spacing * (index - (len(vehicle_ids) - 1) / 2.0))
        offsets.append(offset)
    return offsets


def formation_label(formation_type):
    for label, value in FORMATION_OPTIONS:
        if value == formation_type:
            return label
    return formation_type or "-"


def route_mode_label(route_mode):
    mapping = {
        "direct": "直达",
        "formation_preserving": "编队保持绕行",
        "corridor_compressing": "通道压缩通过",
        "single_file_regrouping": "单列穿越重组",
        "vehicle_deformed": "局部变形绕行",
    }
    return mapping.get(route_mode or "direct", route_mode or "直达")


class SituationView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(980, 680)
        self.scale_m_per_px = 1.2
        self.view_center = (0.0, 0.0)
        self.anchor = (0.0, 0.0)
        self.safe_anchor = (0.0, 0.0)
        self.heading_deg = 90.0
        self.formation_type = "line"
        self.spacing = 35.0
        self.positions = {}
        self.obstacles = {}
        self.avoidance_state = "clear"
        self.requested_path = []
        self.planned_path = []
        self.regroup_points = []
        self.staging_points = []
        self.staging_path = []
        self.vehicle_errors = {}
        self.layering_zone = None
        self.altitude_assignments = {}
        self.altitude_levels = {}
        self.comparison_routes = {}
        self.vehicle_layer_tags = {}
        self.route_mode = "direct"
        self.anchor_callback = None
        self.field_selection_callback = None
        self.last_click_xy = None
        self._dragging = False
        self._drag_start = None
        self._drag_origin = (0.0, 0.0)
        self.field_selection_mode = False
        self.field_region = None
        self._selecting_field = False
        self._selection_start_world = None
        self._selection_end_world = None

    def set_anchor_callback(self, callback):
        self.anchor_callback = callback

    def set_field_selection_callback(self, callback):
        self.field_selection_callback = callback

    def set_field_selection_mode(self, enabled):
        self.field_selection_mode = bool(enabled)
        self._selecting_field = False
        self._selection_start_world = None
        self._selection_end_world = None
        self.update()

    def update_scene(
        self,
        positions,
        obstacles,
        formation_type,
        spacing,
        heading_deg,
        anchor,
        safe_anchor,
        avoidance_state,
        requested_path=None,
        planned_path=None,
        regroup_points=None,
        staging_points=None,
        staging_path=None,
        layering_zone=None,
        altitude_assignments=None,
        altitude_levels=None,
        comparison_routes=None,
        vehicle_layer_tags=None,
        field_region=None,
        route_mode="direct",
    ):
        self.positions = dict(positions)
        self.obstacles = dict(obstacles)
        self.formation_type = formation_type
        self.spacing = spacing
        self.heading_deg = heading_deg
        self.anchor = anchor
        self.safe_anchor = safe_anchor
        self.avoidance_state = avoidance_state
        self.requested_path = list(requested_path or [])
        self.planned_path = list(planned_path or [])
        self.regroup_points = list(regroup_points or [])
        self.staging_points = list(staging_points or [])
        self.staging_path = list(staging_path or staging_points or [])
        self.vehicle_errors = dict(getattr(self, "vehicle_errors", {}))
        self.layering_zone = dict(layering_zone) if layering_zone else None
        self.altitude_assignments = dict(altitude_assignments or {})
        self.altitude_levels = dict(altitude_levels or {})
        self.comparison_routes = dict(comparison_routes or {})
        self.vehicle_layer_tags = dict(vehicle_layer_tags or {})
        self.field_region = dict(field_region) if field_region else None
        self.route_mode = route_mode or "direct"
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            world_x, world_y = self._pixel_to_world(event.x(), event.y())
            if self.field_selection_mode:
                self._selecting_field = True
                self._selection_start_world = (world_x, world_y)
                self._selection_end_world = (world_x, world_y)
                self.update()
                return
            self.last_click_xy = (world_x, world_y)
            if self.anchor_callback is not None:
                self.anchor_callback(world_x, world_y)
            self.update()
        elif event.button() == Qt.RightButton:
            self._dragging = True
            self._drag_start = (event.x(), event.y())
            self._drag_origin = self.view_center
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._selecting_field:
            world_x, world_y = self._pixel_to_world(event.x(), event.y())
            self._selection_end_world = (world_x, world_y)
            self.update()
            return
        if not self._dragging or self._drag_start is None:
            return
        dx_px = event.x() - self._drag_start[0]
        dy_px = event.y() - self._drag_start[1]
        self.view_center = (
            self._drag_origin[0] - dx_px * self.scale_m_per_px,
            self._drag_origin[1] + dy_px * self.scale_m_per_px,
        )
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._selecting_field:
            self._selecting_field = False
            if self._selection_start_world is not None and self._selection_end_world is not None:
                x1, y1 = self._selection_start_world
                x2, y2 = self._selection_end_world
                min_x, max_x = min(x1, x2), max(x1, x2)
                min_y, max_y = min(y1, y2), max(y1, y2)
                if abs(max_x - min_x) > 5.0 and abs(max_y - min_y) > 5.0:
                    self.field_region = {
                        "center_x": (min_x + max_x) / 2.0,
                        "center_y": (min_y + max_y) / 2.0,
                        "width": max_x - min_x,
                        "height": max_y - min_y,
                    }
                    if self.field_selection_callback is not None:
                        self.field_selection_callback(
                            self.field_region["center_x"],
                            self.field_region["center_y"],
                            self.field_region["width"],
                            self.field_region["height"],
                        )
            self._selection_start_world = None
            self._selection_end_world = None
            self.update()
            return
        if event.button() == Qt.RightButton:
            self._dragging = False
            self._drag_start = None
            self.unsetCursor()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        zoom_factor = 0.88 if delta > 0 else 1.14
        self.scale_m_per_px = min(max(self.scale_m_per_px * zoom_factor, 0.2), 20.0)
        self.update()

    def reset_view(self):
        self.view_center = (0.0, 0.0)
        self.scale_m_per_px = 1.2
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#09111f"))
        self._draw_grid(painter)
        self._draw_obstacles(painter)
        self._draw_anchor(painter, self.anchor, QColor("#ffd166"), "任务锚点")
        if self.safe_anchor != self.anchor:
            self._draw_anchor(painter, self.safe_anchor, QColor("#ef476f"), "避障锚点")
        self._draw_paths(painter)
        self._draw_field_region(painter)
        self._draw_layering_overlay(painter)
        self._draw_formation(painter, self.anchor, dashed=True)
        self._draw_formation(painter, self.safe_anchor, dashed=False)
        self._draw_vehicles(painter)
        self._draw_legend(painter)

    def _draw_grid(self, painter):
        width = self.width()
        height = self.height()
        minor_pen = QPen(QColor(30, 50, 75))
        major_pen = QPen(QColor(48, 82, 120))
        axis_pen = QPen(QColor(100, 150, 200))
        axis_pen.setWidth(2)
        grid_spacing_m = 50.0
        left_world, top_world = self._pixel_to_world(0, 0)
        right_world, bottom_world = self._pixel_to_world(width, height)
        min_x = min(left_world, right_world)
        max_x = max(left_world, right_world)
        min_y = min(bottom_world, top_world)
        max_y = max(bottom_world, top_world)

        start_x = int(math.floor(min_x / grid_spacing_m)) * int(grid_spacing_m)
        end_x = int(math.ceil(max_x / grid_spacing_m)) * int(grid_spacing_m)
        start_y = int(math.floor(min_y / grid_spacing_m)) * int(grid_spacing_m)
        end_y = int(math.ceil(max_y / grid_spacing_m)) * int(grid_spacing_m)

        for x_value in range(start_x, end_x + int(grid_spacing_m), int(grid_spacing_m)):
            point = self._world_to_pixel(x_value, 0.0)
            painter.setPen(major_pen if (x_value // int(grid_spacing_m)) % 2 == 0 else minor_pen)
            painter.drawLine(int(point.x()), 0, int(point.x()), height)
        for y_value in range(start_y, end_y + int(grid_spacing_m), int(grid_spacing_m)):
            point = self._world_to_pixel(0.0, y_value)
            painter.setPen(major_pen if (y_value // int(grid_spacing_m)) % 2 == 0 else minor_pen)
            painter.drawLine(0, int(point.y()), width, int(point.y()))

        painter.setPen(axis_pen)
        zero_x = self._world_to_pixel(0.0, 0.0).x()
        zero_y = self._world_to_pixel(0.0, 0.0).y()
        painter.drawLine(0, int(zero_y), width, int(zero_y))
        painter.drawLine(int(zero_x), 0, int(zero_x), height)

    def _draw_obstacles(self, painter):
        for obstacle in self.obstacles.values():
            if not obstacle.enabled:
                continue
            center = self._world_to_pixel(obstacle.pose.position.x, obstacle.pose.position.y)
            half_w = obstacle.size.x / self.scale_m_per_px / 2.0
            half_h = obstacle.size.y / self.scale_m_per_px / 2.0
            rect = QRectF(center.x() - half_w, center.y() - half_h, half_w * 2.0, half_h * 2.0)
            painter.setPen(QPen(QColor("#ff7b72"), 2))
            painter.setBrush(QBrush(QColor(255, 123, 114, 90)))
            if obstacle.shape == "cylinder":
                painter.drawEllipse(rect)
            else:
                painter.drawRoundedRect(rect, 6.0, 6.0)
            painter.setPen(QColor("#ffd7d5"))
            painter.drawText(rect.topLeft() + QPointF(4.0, -4.0), obstacle.id.replace("obstacle_", "障碍"))

    def _draw_anchor(self, painter, anchor, color, label):
        point = self._world_to_pixel(anchor[0], anchor[1])
        painter.setPen(QPen(color, 2))
        painter.drawLine(point.x() - 10, point.y(), point.x() + 10, point.y())
        painter.drawLine(point.x(), point.y() - 10, point.x(), point.y() + 10)
        painter.drawText(point + QPointF(12.0, -8.0), label)

    def _draw_field_region(self, painter):
        region = self.field_region
        if self._selecting_field and self._selection_start_world and self._selection_end_world:
            x1, y1 = self._selection_start_world
            x2, y2 = self._selection_end_world
            region = {
                "center_x": (x1 + x2) / 2.0,
                "center_y": (y1 + y2) / 2.0,
                "width": abs(x2 - x1),
                "height": abs(y2 - y1),
            }
        if not region:
            return
        center = self._world_to_pixel(region["center_x"], region["center_y"])
        half_w = region["width"] / self.scale_m_per_px / 2.0
        half_h = region["height"] / self.scale_m_per_px / 2.0
        rect = QRectF(center.x() - half_w, center.y() - half_h, half_w * 2.0, half_h * 2.0)
        pen = QPen(QColor("#80ed99"), 2)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(rect)
        painter.drawText(rect.topLeft() + QPointF(4.0, -6.0), "障碍区域")

    def _draw_paths(self, painter):
        if len(self.requested_path) >= 2:
            pen = QPen(QColor("#8ecae6"), 2)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            for first, second in zip(self.requested_path[:-1], self.requested_path[1:]):
                start = self._world_to_pixel(first["x"], first["y"])
                end = self._world_to_pixel(second["x"], second["y"])
                painter.drawLine(start, end)
            for point in self.requested_path:
                pixel = self._world_to_pixel(point["x"], point["y"])
                painter.setBrush(QBrush(QColor("#8ecae6")))
                painter.drawEllipse(pixel, 4, 4)

        if len(self.planned_path) >= 2:
            pen = QPen(QColor("#ffb703"), 3)
            painter.setPen(pen)
            for first, second in zip(self.planned_path[:-1], self.planned_path[1:]):
                start = self._world_to_pixel(first["x"], first["y"])
                end = self._world_to_pixel(second["x"], second["y"])
                painter.drawLine(start, end)
            for index, point in enumerate(self.planned_path):
                pixel = self._world_to_pixel(point["x"], point["y"])
                painter.setBrush(QBrush(QColor("#ffb703")))
                painter.drawEllipse(pixel, 5, 5)
                painter.drawText(pixel + QPointF(6.0, -6.0), "P{}".format(index + 1))

        if self.regroup_points:
            painter.setPen(QPen(QColor("#80ed99"), 2))
            painter.setBrush(QBrush(QColor("#80ed99")))
            for index, point in enumerate(self.regroup_points):
                pixel = self._world_to_pixel(point["x"], point["y"])
                painter.drawEllipse(pixel, 6, 6)
                painter.drawText(pixel + QPointF(8.0, -10.0), "R{}".format(index + 1))

        if self.staging_points:
            stage_pen = QPen(QColor("#c77dff"), 2)
            stage_pen.setStyle(Qt.DashLine)
            painter.setPen(stage_pen)
            stage_polyline = self.staging_path if len(self.staging_path) >= 2 else self.staging_points
            for first, second in zip(stage_polyline[:-1], stage_polyline[1:]):
                start = self._world_to_pixel(first["x"], first["y"])
                end = self._world_to_pixel(second["x"], second["y"])
                painter.drawLine(start, end)
            painter.setBrush(QBrush(QColor("#c77dff")))
            painter.setPen(QPen(QColor("#e9d5ff"), 2))
            for index, point in enumerate(self.staging_points):
                pixel = self._world_to_pixel(point["x"], point["y"])
                radius = 8 if index == len(self.staging_points) - 1 else 7
                painter.drawEllipse(pixel, radius, radius)
                painter.drawText(pixel + QPointF(10.0, -10.0), "S{}".format(index + 1))

    def _draw_layering_overlay(self, painter):
        zone = self.layering_zone
        if zone:
            center = self._world_to_pixel(zone["center_x"], zone["center_y"])
            half_w = zone["width"] / self.scale_m_per_px / 2.0
            half_h = zone["height"] / self.scale_m_per_px / 2.0
            rect = QRectF(center.x() - half_w, center.y() - half_h, half_w * 2.0, half_h * 2.0)
            pen = QPen(QColor("#b388ff"), 2)
            pen.setStyle(Qt.DashDotLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(rect, 8.0, 8.0)
            painter.drawText(rect.topLeft() + QPointF(4.0, -6.0), "分层重组区")

        route_specs = [
            ("unlayered", QColor("#adb5bd"), "未分层参考"),
            ("layered", QColor("#4cc9f0"), "分层后参考"),
        ]
        for key, color, label in route_specs:
            route = self.comparison_routes.get(key) or []
            if len(route) < 2:
                continue
            pen = QPen(color, 2)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            for first, second in zip(route[:-1], route[1:]):
                start = self._world_to_pixel(first["x"], first["y"])
                end = self._world_to_pixel(second["x"], second["y"])
                painter.drawLine(start, end)
            end_pixel = self._world_to_pixel(route[-1]["x"], route[-1]["y"])
            painter.drawText(end_pixel + QPointF(10.0, -6.0), label)

        if self.altitude_assignments and self.altitude_levels:
            base_level = float(self.altitude_levels.get("base", self.anchor[2] if len(self.anchor) > 2 else 0.0))
            rows = []
            for layer_key, color, prefix in [
                ("upper", QColor("#4cc9f0"), "上层"),
                ("base", QColor("#06d6a0"), "基准层"),
                ("lower", QColor("#f4a261"), "下层"),
            ]:
                vehicle_ids = list(self.altitude_assignments.get(layer_key, []))
                if not vehicle_ids:
                    continue
                level = float(self.altitude_levels.get(layer_key, base_level))
                delta = level - base_level
                if abs(delta) < 1e-6:
                    tag = "H"
                elif delta > 0:
                    tag = "H+{:.0f}m".format(delta)
                else:
                    tag = "H{:.0f}m".format(delta)
                rows.append((color, "{} {}: {}".format(prefix, tag, ", ".join(vehicle_ids))))
            if rows:
                origin = QPointF(22.0, float(self.height() - 78 - 18 * len(rows)))
                for row_index, (color, text) in enumerate(rows):
                    y = origin.y() + 18.0 * row_index
                    painter.setPen(QPen(color, 2))
                    painter.drawLine(QPointF(origin.x(), y), QPointF(origin.x() + 14.0, y))
                    painter.setPen(QColor("#d8f3ff"))
                    painter.drawText(QPointF(origin.x() + 20.0, y + 4.0), text)

    def _draw_formation(self, painter, anchor, dashed):
        expected_positions = compute_formation_positions(
            self.formation_type,
            self.spacing,
            self.heading_deg,
            anchor,
            formation_vehicle_ids(self.formation_type, VEHICLE_IDS),
        )
        pen = QPen(QColor("#94d2bd") if dashed else QColor("#06d6a0"), 2)
        pen.setStyle(Qt.DashLine if dashed else Qt.SolidLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        for vehicle_id, (x_value, y_value) in expected_positions.items():
            point = self._world_to_pixel(x_value, y_value)
            painter.drawEllipse(point, 7, 7)
            painter.drawText(point + QPointF(8.0, -8.0), vehicle_id)

    def _draw_vehicles(self, painter):
        visible_vehicle_ids = formation_vehicle_ids(self.formation_type, VEHICLE_IDS)
        for index, vehicle_id in enumerate(visible_vehicle_ids):
            if vehicle_id not in self.positions:
                continue
            point = self._world_to_pixel(self.positions[vehicle_id][0], self.positions[vehicle_id][1])
            color = QColor(VEHICLE_COLORS[index % len(VEHICLE_COLORS)])
            painter.setPen(QPen(color, 1))
            painter.setBrush(QBrush(color))
            painter.drawPolygon(
                QPolygonF(
                    [
                        QPointF(point.x(), point.y() - 10.0),
                        QPointF(point.x() + 7.0, point.y() + 8.0),
                        QPointF(point.x(), point.y() + 4.0),
                        QPointF(point.x() - 7.0, point.y() + 8.0),
                    ]
                )
            )
            painter.drawText(point + QPointF(10.0, 14.0), vehicle_id)
            layer_tag = self.vehicle_layer_tags.get(vehicle_id)
            if layer_tag:
                painter.setPen(QPen(QColor("#d8f3ff"), 1))
                painter.drawText(point + QPointF(10.0, -12.0), layer_tag)
            vehicle_error = float(self.vehicle_errors.get(vehicle_id, 0.0))
            if vehicle_error >= 12.0:
                painter.setPen(QPen(QColor("#ffd166"), 1))
                painter.drawText(point + QPointF(10.0, 30.0), "e={:.0f}m".format(vehicle_error))

    def _draw_legend(self, painter):
        painter.setPen(QColor("#d8f3ff"))
        painter.drawText(
            QPointF(18.0, 24.0),
            "左键设锚点{} | 右键拖动画布 | 滚轮缩放 | 蓝虚线=请求航路 | 黄实线=规划航路 | 避障状态={} | 规划模式={}".format(
                " / 框选障碍区域" if self.field_selection_mode else "",
                self.avoidance_state,
                route_mode_label(self.route_mode),
            ),
        )
        if self.last_click_xy is not None:
            painter.drawText(
                QPointF(18.0, 44.0),
                "最近点击: x={:.1f} m, y={:.1f} m".format(self.last_click_xy[0], self.last_click_xy[1]),
            )
        painter.drawText(
            QPointF(18.0, 64.0),
            "视图中心: x={:.1f} m, y={:.1f} m | 比例: {:.2f} 米/像素".format(
                self.view_center[0], self.view_center[1], self.scale_m_per_px
            ),
        )

    def _world_to_pixel(self, x_value, y_value):
        center_x = self.width() / 2.0
        center_y = self.height() / 2.0
        return QPointF(
            center_x + (x_value - self.view_center[0]) / self.scale_m_per_px,
            center_y - (y_value - self.view_center[1]) / self.scale_m_per_px,
        )

    def _pixel_to_world(self, pixel_x, pixel_y):
        center_x = self.width() / 2.0
        center_y = self.height() / 2.0
        return (
            self.view_center[0] + (pixel_x - center_x) * self.scale_m_per_px,
            self.view_center[1] - (pixel_y - center_y) * self.scale_m_per_px,
        )


class SwarmControlWidget(QWidget):
    status_signal = Signal(object)
    formation_signal = Signal(object)
    safe_formation_signal = Signal(object)
    obstacle_signal = Signal(object)
    avoidance_signal = Signal(object)
    gazebo_signal = Signal(str)
    pose_signal = Signal(str, float, float, float)
    action_result_signal = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("固定翼集群控制台")
        self._apply_cjk_font()
        self.setMinimumSize(1500, 920)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._status_label = QLabel("集群状态: 未知")
        self._workflow_label = QLabel("任务流程: 未配置")
        self._gazebo_label = QLabel("Gazebo联动: 未知")
        self._workflow_detail_label = None
        self._vehicle_table = None
        self._situation_label = None
        self._situation_view = None
        self._formation_pub = rospy.Publisher("/swarm/formation_cmd", FormationCommand, queue_size=10, latch=True)
        self._obstacle_pub = rospy.Publisher("/swarm/obstacles", Obstacle, queue_size=10)
        self._mission_anchor_pub = rospy.Publisher("/swarm/mission_anchor", PoseStamped, queue_size=10, latch=True)
        self._mission_phase_pub = rospy.Publisher("/swarm/mission_phase_cmd", String, queue_size=10, latch=True)
        self._status_sub = rospy.Subscriber("/swarm/status", SwarmStatus, self._on_status)
        self._formation_sub = rospy.Subscriber("/swarm/formation_cmd", FormationCommand, self._on_formation_cmd)
        self._safe_formation_sub = rospy.Subscriber("/swarm/formation_cmd_safe", FormationCommand, self._on_safe_formation_cmd)
        self._obstacle_sub = rospy.Subscriber("/swarm/obstacles", Obstacle, self._on_obstacle)
        self._avoidance_status_sub = rospy.Subscriber("/swarm/avoidance/state", SwarmStatus, self._on_avoidance_status)
        self._gazebo_status_sub = rospy.Subscriber("/swarm/gazebo_sync_status", String, self._on_gazebo_status)

        self._vehicle_positions = {}
        self._vehicle_last_seen = {}
        self._pose_subscribers = []
        self._obstacles = {}
        self._obstacle_order = []
        self._obstacle_sequence = 0
        self._active_formation = "line"
        self._active_spacing = 35.0
        self._active_heading = 90.0
        self._anchor_xy = (0.0, 0.0)
        self._safe_anchor_xy = (0.0, 0.0)
        self._avoidance_state = "clear"
        self._task_points = []
        self._mission_phase = "idle"
        self._mission_refresh_lock = threading.Lock()
        self._mission_refresh_pending = False
        self._mission_refresh_reason = None
        self._last_refresh_request_time = 0.0
        self._arming_clients = {}
        self._mode_clients = {}
        self._command_clients = {}
        self._param_set_clients = {}
        self._waypoint_clear_clients = {}
        self._waypoint_push_clients = {}
        self._mission_counts = {}
        self._global_positions = {}
        self._global_subscribers = []
        self._requested_task_path = []
        self._planned_task_path = []
        self._regroup_points = []
        self._launch_staging_points = []
        self._launch_staging_path = []
        self._plan_vehicle_routes = {}
        self._latest_plan_summary = {}
        self._route_mode = "direct"
        self._planner_mode = "adaptive"
        self._scenario_key = "urban_delivery"
        self._scenario_layering_zone = None
        self._scenario_altitude_levels = {}
        self._scenario_altitude_assignments = {}
        self._scenario_comparison_routes = {}
        self._mission_refresh_count = 0
        self._formation_error_live = 0.0
        self._live_vehicle_formation_errors = {}
        self._obstacle_clearance_live = float("inf")
        self._formation_switch_attempts = 0
        self._formation_switch_successes = 0
        self._formation_switch_started_at = None
        self._formation_switch_target = None
        self._formation_switch_recovery_times = []
        self._last_export_path = ""
        self._metric_label = None
        self._scenario_detail_label = None
        self._workspace_root = workspace_root_from_file(__file__)
        self._results_dir = self._workspace_root / "results" / "swarm_experiments"
        self._results_dir.mkdir(parents=True, exist_ok=True)
        self._vehicle_body_radius = float(rospy.get_param("~vehicle_body_radius_m", DEFAULT_FIXED_WING_BODY_RADIUS_M))
        self._vehicle_tracking_margin = float(
            rospy.get_param("~vehicle_tracking_margin_m", DEFAULT_FIXED_WING_TRACKING_MARGIN_M)
        )
        self._vehicle_turn_buffer = float(rospy.get_param("~vehicle_turn_buffer_m", DEFAULT_FIXED_WING_TURN_BUFFER_M))
        self._enable_prearm_param_push = bool(rospy.get_param("~enable_prearm_param_push", False))

        self.status_signal.connect(self._handle_status_update)
        self.formation_signal.connect(self._handle_formation_update)
        self.safe_formation_signal.connect(self._handle_safe_formation_update)
        self.obstacle_signal.connect(self._handle_obstacle_update)
        self.avoidance_signal.connect(self._handle_avoidance_update)
        self.gazebo_signal.connect(self._handle_gazebo_status)
        self.pose_signal.connect(self._handle_pose_update)
        self.action_result_signal.connect(self._status_label.setText)

        for vehicle_id in VEHICLE_IDS:
            topic = "/{}/mavros/local_position/pose".format(vehicle_id)
            self._pose_subscribers.append(
                rospy.Subscriber(topic, PoseStamped, self._make_pose_callback(vehicle_id), queue_size=1)
            )
            self._global_subscribers.append(
                rospy.Subscriber(
                    "/{}/mavros/global_position/global".format(vehicle_id),
                    NavSatFix,
                    self._make_global_callback(vehicle_id),
                    queue_size=1,
                )
            )

        self._build_ui()
        self._update_route_preview()
        self._update_workflow_status()

    def _apply_cjk_font(self):
        preferred_families = [
            "Noto Sans CJK SC",
            "Noto Sans CJK JP",
            "WenQuanYi Zen Hei",
            "Microsoft YaHei",
            "SimHei",
        ]
        widget_font = QFont()
        widget_font.setPointSize(11)
        for family in preferred_families:
            candidate = QFont(family, 11)
            if candidate.exactMatch():
                widget_font = candidate
                break
        self.setFont(widget_font)

    def _build_ui(self):
        root = QHBoxLayout()
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)

        left_container = QWidget()
        left_container.setMinimumWidth(460)
        left_container.setMaximumWidth(540)
        left_panel = QVBoxLayout(left_container)
        left_panel.setContentsMargins(0, 0, 0, 0)
        left_panel.setSpacing(12)
        left_panel.setAlignment(Qt.AlignTop)
        left_panel.addWidget(self._status_label)
        left_panel.addWidget(self._workflow_label)
        left_panel.addWidget(self._gazebo_label)

        formation_group = QGroupBox("编队参数")
        formation_layout = QFormLayout()
        formation_layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        formation_layout.setFormAlignment(Qt.AlignTop)
        formation_layout.setHorizontalSpacing(10)
        formation_layout.setVerticalSpacing(8)
        self._formation_type = QComboBox()
        for label, value in FORMATION_OPTIONS:
            self._formation_type.addItem(label, value)
        self._spacing = QDoubleSpinBox()
        self._spacing.setRange(5.0, 300.0)
        self._spacing.setValue(35.0)
        self._spacing.setSuffix(" 米")
        self._heading = QDoubleSpinBox()
        self._heading.setRange(0.0, 360.0)
        self._heading.setValue(90.0)
        self._heading.setSuffix(" 度")
        self._anchor_x = QDoubleSpinBox()
        self._anchor_y = QDoubleSpinBox()
        self._anchor_z = QDoubleSpinBox()
        for widget, value in ((self._anchor_x, 0.0), (self._anchor_y, 0.0), (self._anchor_z, 60.0)):
            widget.setRange(-20000.0, 20000.0)
            widget.setValue(value)
            widget.setDecimals(1)
            widget.setSingleStep(10.0)
        formation_layout.addRow("队形", self._formation_type)
        formation_layout.addRow("间距", self._spacing)
        formation_layout.addRow("航向", self._heading)
        formation_layout.addRow("锚点 X", self._anchor_x)
        formation_layout.addRow("锚点 Y", self._anchor_y)
        formation_layout.addRow("高度", self._anchor_z)
        apply_formation_btn = QPushButton("应用编队")
        apply_formation_btn.clicked.connect(self._publish_formation)
        formation_layout.addRow(apply_formation_btn)
        formation_group.setLayout(formation_layout)

        flight_group = QGroupBox("飞控执行（调试 / QGC）")
        flight_layout = QGridLayout()
        flight_layout.setHorizontalSpacing(8)
        flight_layout.setVerticalSpacing(8)
        arm_btn = QPushButton("全体解锁(调试)")
        arm_btn.clicked.connect(lambda: self._call_arm_service(True))
        disarm_btn = QPushButton("全体上锁(调试)")
        disarm_btn.clicked.connect(lambda: self._call_arm_service(False))
        upload_mission_btn = QPushButton("上传编队任务")
        upload_mission_btn.clicked.connect(self._upload_formation_mission)
        start_mission_btn = QPushButton("开始任务(调试)")
        start_mission_btn.clicked.connect(self._start_mission)
        mission_btn = QPushButton("任务模式(调试)")
        mission_btn.clicked.connect(lambda: self._call_mode_service("AUTO.MISSION"))
        mission_btn.setEnabled(False)
        mission_btn.setToolTip("固定翼主流程请使用“开始任务”，此按钮仅保留给调试场景。")
        loiter_btn = QPushButton("盘旋待命")
        loiter_btn.clicked.connect(lambda: self._call_mode_service("AUTO.LOITER"))
        flight_layout.addWidget(arm_btn, 0, 0)
        flight_layout.addWidget(disarm_btn, 0, 1)
        flight_layout.addWidget(upload_mission_btn, 1, 0, 1, 2)
        flight_layout.addWidget(start_mission_btn, 2, 0, 1, 2)
        flight_layout.addWidget(mission_btn, 3, 0)
        flight_layout.addWidget(loiter_btn, 3, 1)
        flight_group.setLayout(flight_layout)

        obstacle_group = QGroupBox("二维障碍设置")
        obstacle_layout = QGridLayout()
        obstacle_layout.setHorizontalSpacing(8)
        obstacle_layout.setVerticalSpacing(8)
        self._obs_x = QDoubleSpinBox()
        self._obs_y = QDoubleSpinBox()
        self._obs_z = QDoubleSpinBox()
        self._obs_sx = QDoubleSpinBox()
        self._obs_sy = QDoubleSpinBox()
        self._obs_h = QDoubleSpinBox()
        self._obs_field_cx = QDoubleSpinBox()
        self._obs_field_cy = QDoubleSpinBox()
        self._obs_field_w = QDoubleSpinBox()
        self._obs_field_hh = QDoubleSpinBox()
        self._field_select_btn = None
        for widget, value in (
            (self._obs_x, 60.0),
            (self._obs_y, 0.0),
            (self._obs_z, 0.0),
            (self._obs_sx, 20.0),
            (self._obs_sy, 20.0),
            (self._obs_h, 120.0),
            (self._obs_field_cx, 180.0),
            (self._obs_field_cy, 0.0),
            (self._obs_field_w, 320.0),
            (self._obs_field_hh, 240.0),
        ):
            widget.setRange(-20000.0, 20000.0)
            widget.setValue(value)
            widget.setDecimals(1)
        self._obs_shape = QComboBox()
        for label, value in SHAPE_OPTIONS:
            self._obs_shape.addItem(label, value)
        obstacle_layout.addWidget(QLabel("X"), 0, 0)
        obstacle_layout.addWidget(self._obs_x, 0, 1)
        obstacle_layout.addWidget(QLabel("Y"), 0, 2)
        obstacle_layout.addWidget(self._obs_y, 0, 3)
        obstacle_layout.addWidget(QLabel("尺寸 X"), 1, 0)
        obstacle_layout.addWidget(self._obs_sx, 1, 1)
        obstacle_layout.addWidget(QLabel("尺寸 Y"), 1, 2)
        obstacle_layout.addWidget(self._obs_sy, 1, 3)
        obstacle_layout.addWidget(QLabel("底部 Z"), 2, 0)
        obstacle_layout.addWidget(self._obs_z, 2, 1)
        obstacle_layout.addWidget(QLabel("高度"), 2, 2)
        obstacle_layout.addWidget(self._obs_h, 2, 3)
        obstacle_layout.addWidget(QLabel("形状"), 3, 0)
        obstacle_layout.addWidget(self._obs_shape, 3, 1)
        obstacle_layout.addWidget(QLabel("场景中心 X"), 4, 0)
        obstacle_layout.addWidget(self._obs_field_cx, 4, 1)
        obstacle_layout.addWidget(QLabel("场景中心 Y"), 4, 2)
        obstacle_layout.addWidget(self._obs_field_cy, 4, 3)
        obstacle_layout.addWidget(QLabel("范围 X"), 5, 0)
        obstacle_layout.addWidget(self._obs_field_w, 5, 1)
        obstacle_layout.addWidget(QLabel("范围 Y"), 5, 2)
        obstacle_layout.addWidget(self._obs_field_hh, 5, 3)
        self._field_select_btn = QPushButton("框选生成区域")
        self._field_select_btn.setCheckable(True)
        self._field_select_btn.toggled.connect(self._toggle_field_selection_mode)
        add_obstacle_btn = QPushButton("添加障碍物")
        add_obstacle_btn.clicked.connect(self._publish_obstacle)
        obstacle_layout.addWidget(self._field_select_btn, 6, 0, 1, 2)
        obstacle_layout.addWidget(add_obstacle_btn, 6, 2, 1, 2)
        random_10_btn = QPushButton("随机 10 个")
        random_10_btn.clicked.connect(lambda: self._generate_random_obstacles(10))
        random_20_btn = QPushButton("随机 20 个")
        random_20_btn.clicked.connect(lambda: self._generate_random_obstacles(20))
        obstacle_layout.addWidget(random_10_btn, 7, 0, 1, 2)
        obstacle_layout.addWidget(random_20_btn, 7, 2, 1, 2)
        sparse_field_btn = QPushButton("稀疏场")
        sparse_field_btn.clicked.connect(lambda: self._generate_obstacle_field("sparse"))
        dense_field_btn = QPushButton("密集场")
        dense_field_btn.clicked.connect(lambda: self._generate_obstacle_field("dense"))
        corridor_field_btn = QPushButton("通道场")
        corridor_field_btn.clicked.connect(lambda: self._generate_obstacle_field("corridor"))
        obstacle_layout.addWidget(sparse_field_btn, 8, 0, 1, 1)
        obstacle_layout.addWidget(dense_field_btn, 8, 1, 1, 1)
        obstacle_layout.addWidget(corridor_field_btn, 8, 2, 1, 2)
        checkerboard_field_btn = QPushButton("棋盘场")
        checkerboard_field_btn.clicked.connect(lambda: self._generate_obstacle_field("checkerboard"))
        s_curve_field_btn = QPushButton("S 型通道")
        s_curve_field_btn.clicked.connect(lambda: self._generate_obstacle_field("s_curve"))
        obstacle_layout.addWidget(checkerboard_field_btn, 9, 0, 1, 2)
        obstacle_layout.addWidget(s_curve_field_btn, 9, 2, 1, 2)
        remove_last_obstacle_btn = QPushButton("删除最后一个")
        remove_last_obstacle_btn.clicked.connect(self._remove_last_obstacle)
        clear_obstacles_btn = QPushButton("清空障碍物")
        clear_obstacles_btn.clicked.connect(self._clear_obstacles)
        obstacle_layout.addWidget(remove_last_obstacle_btn, 10, 0, 1, 2)
        obstacle_layout.addWidget(clear_obstacles_btn, 10, 2, 1, 2)
        obstacle_group.setLayout(obstacle_layout)

        mission_group = QGroupBox("编队参考点（高级）")
        mission_layout = QHBoxLayout()
        mission_layout.setSpacing(8)
        publish_anchor_btn = QPushButton("仅更新参考点")
        publish_anchor_btn.clicked.connect(self._publish_anchor)
        focus_swarm_btn = QPushButton("对准当前机群")
        focus_swarm_btn.clicked.connect(self._center_on_swarm)
        reset_view_btn = QPushButton("重置视图")
        reset_view_btn.clicked.connect(self._reset_canvas_view)
        mission_layout.addWidget(publish_anchor_btn)
        mission_layout.addWidget(focus_swarm_btn)
        mission_layout.addWidget(reset_view_btn)
        mission_group.setLayout(mission_layout)

        task_group = QGroupBox("共同航点")
        task_layout = QVBoxLayout()
        task_layout.setSpacing(8)
        task_help = QLabel("共同航点表示整队共享的航线点；系统会结合当前队形，自动为每架飞机生成各自任务。")
        task_help.setWordWrap(True)
        task_form = QGridLayout()
        task_form.setHorizontalSpacing(8)
        task_form.setVerticalSpacing(8)
        self._task_x = QDoubleSpinBox()
        self._task_y = QDoubleSpinBox()
        self._task_alt = QDoubleSpinBox()
        for widget, value in ((self._task_x, 120.0), (self._task_y, 0.0), (self._task_alt, 80.0)):
            widget.setRange(-20000.0, 20000.0)
            widget.setValue(value)
            widget.setDecimals(1)
            widget.setSingleStep(10.0)
        task_form.addWidget(QLabel("X"), 0, 0)
        task_form.addWidget(self._task_x, 0, 1)
        task_form.addWidget(QLabel("Y"), 0, 2)
        task_form.addWidget(self._task_y, 0, 3)
        task_form.addWidget(QLabel("高度"), 1, 0)
        task_form.addWidget(self._task_alt, 1, 1)
        task_layout.addWidget(task_help)
        task_layout.addLayout(task_form)
        task_btns = QHBoxLayout()
        task_btns.setSpacing(8)
        add_task_btn = QPushButton("添加任务点")
        add_task_btn.clicked.connect(self._add_task_point)
        pop_task_btn = QPushButton("删除最后一个")
        pop_task_btn.clicked.connect(self._remove_last_task_point)
        clear_task_btn = QPushButton("清空任务点")
        clear_task_btn.clicked.connect(self._clear_task_points)
        task_btns.addWidget(add_task_btn)
        task_btns.addWidget(pop_task_btn)
        task_btns.addWidget(clear_task_btn)
        self._task_table = QTableWidget(0, 4)
        self._task_table.setHorizontalHeaderLabels(["序号", "X", "Y", "高度"])
        self._task_table.verticalHeader().setVisible(False)
        self._task_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._task_table.setSelectionMode(QTableWidget.NoSelection)
        self._task_table.setMinimumHeight(180)
        task_layout.addLayout(task_btns)
        task_layout.addWidget(self._task_table)
        task_group.setLayout(task_layout)

        experiment_group = QGroupBox("实验评估")
        experiment_layout = QVBoxLayout()
        experiment_layout.setSpacing(8)
        experiment_form = QFormLayout()
        experiment_form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        experiment_form.setFormAlignment(Qt.AlignTop)
        experiment_form.setHorizontalSpacing(10)
        experiment_form.setVerticalSpacing(8)
        self._scenario_selector = QComboBox()
        for label, value in scenario_options():
            self._scenario_selector.addItem(label, value)
        self._scenario_selector.currentIndexChanged.connect(self._update_scenario_detail)
        self._planner_mode_combo = QComboBox()
        for label, value in PLANNER_MODE_OPTIONS:
            self._planner_mode_combo.addItem(label, value)
        self._planner_mode_combo.currentIndexChanged.connect(self._on_planner_mode_changed)
        experiment_form.addRow("典型场景", self._scenario_selector)
        experiment_form.addRow("对比模式", self._planner_mode_combo)
        experiment_layout.addLayout(experiment_form)
        experiment_btns = QHBoxLayout()
        experiment_btns.setSpacing(8)
        apply_scenario_btn = QPushButton("应用场景")
        apply_scenario_btn.clicked.connect(self._apply_selected_scenario)
        export_metrics_btn = QPushButton("导出实验快照")
        export_metrics_btn.clicked.connect(self._export_experiment_snapshot)
        experiment_btns.addWidget(apply_scenario_btn)
        experiment_btns.addWidget(export_metrics_btn)
        self._scenario_detail_label = QLabel("")
        self._scenario_detail_label.setWordWrap(True)
        self._metric_label = QLabel("")
        self._metric_label.setWordWrap(True)
        experiment_layout.addLayout(experiment_btns)
        experiment_layout.addWidget(self._scenario_detail_label)
        experiment_layout.addWidget(self._metric_label)
        experiment_group.setLayout(experiment_layout)

        self._vehicle_table = QTableWidget(6, 5)
        self._vehicle_table.setHorizontalHeaderLabels(["飞机", "状态", "队形", "X", "Y"])
        self._vehicle_table.setMinimumHeight(220)
        self._vehicle_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._vehicle_table.verticalHeader().setVisible(False)
        self._vehicle_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._vehicle_table.setSelectionMode(QTableWidget.NoSelection)
        for row, vehicle_name in enumerate(VEHICLE_IDS):
            self._vehicle_table.setItem(row, 0, QTableWidgetItem(vehicle_name))
            for column in range(1, 5):
                self._vehicle_table.setItem(row, column, QTableWidgetItem("-"))

        left_panel.addWidget(formation_group)
        left_panel.addWidget(flight_group)
        workflow_group = QGroupBox("任务流程")
        workflow_layout = QVBoxLayout()
        workflow_layout.setSpacing(8)
        workflow_hint = QLabel("建议顺序: 1. 设置共同航点  2. 设置队形/障碍物  3. 上传编队任务  4. 在 QGC 或调试按钮里执行解锁与开始任务")
        workflow_hint.setWordWrap(True)
        self._workflow_detail_label = QLabel("")
        self._workflow_detail_label.setWordWrap(True)
        workflow_layout.addWidget(workflow_hint)
        workflow_layout.addWidget(self._workflow_detail_label)
        workflow_group.setLayout(workflow_layout)

        qgc_hint = QLabel("推荐分工: 本界面负责编队/障碍/任务生成；QGC 负责标准飞控执行与遥测监视。")
        qgc_hint.setWordWrap(True)
        left_panel.addWidget(workflow_group)
        left_panel.addWidget(experiment_group)
        left_panel.addWidget(qgc_hint)
        left_panel.addWidget(obstacle_group)
        left_panel.addWidget(mission_group)
        left_panel.addWidget(task_group)
        left_panel.addWidget(QLabel("飞机状态"))
        left_panel.addWidget(self._vehicle_table)
        left_panel.addStretch(1)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QScrollArea.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setWidget(left_container)

        right_panel = QVBoxLayout()
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(10)
        right_panel.setAlignment(Qt.AlignTop)
        self._situation_label = QLabel("二维态势图")
        self._situation_label.setAlignment(Qt.AlignCenter)
        self._situation_view = SituationView()
        self._situation_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._situation_view.set_anchor_callback(self._set_anchor_from_canvas)
        self._situation_view.set_field_selection_callback(self._set_obstacle_field_region)
        right_panel.addWidget(self._situation_label)
        right_panel.addWidget(self._situation_view, 1)

        root.addWidget(left_scroll, 0)
        root.addLayout(right_panel, 3)
        self.setLayout(root)
        self._refresh_situation_view()
        self._update_scenario_detail()
        self._update_metric_summary()

    def _make_pose_callback(self, vehicle_id):
        def _callback(msg):
            self.pose_signal.emit(vehicle_id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        return _callback

    def _make_global_callback(self, vehicle_id):
        def _callback(msg):
            self._global_positions[vehicle_id] = msg

        return _callback

    def _build_obstacle_message_from_spec(self, spec):
        msg = Obstacle()
        msg.id = spec["id"]
        msg.shape = spec.get("shape", "box")
        msg.enabled = spec.get("enabled", True)
        msg.pose.position.x = float(spec["x"])
        msg.pose.position.y = float(spec["y"])
        obstacle_height = float(spec.get("h", 120.0))
        obstacle_bottom_z = float(spec.get("z", 0.0))
        msg.pose.position.z = obstacle_bottom_z + obstacle_height / 2.0
        msg.pose.orientation.w = 1.0
        msg.size.x = float(spec["sx"])
        msg.size.y = float(spec["sy"])
        msg.size.z = obstacle_height
        return msg

    def _obstacle_to_spec(self, obstacle):
        return {
            "id": obstacle.id,
            "shape": obstacle.shape,
            "enabled": bool(obstacle.enabled),
            "x": float(obstacle.pose.position.x),
            "y": float(obstacle.pose.position.y),
            "z": float(obstacle.pose.position.z - obstacle.size.z / 2.0),
            "sx": float(obstacle.size.x),
            "sy": float(obstacle.size.y),
            "h": float(obstacle.size.z),
        }

    def _active_obstacle_specs(self):
        return [self._obstacle_to_spec(obstacle) for obstacle in self._obstacles.values() if obstacle.enabled]

    def _on_planner_mode_changed(self, _index=None):
        if not hasattr(self, "_planner_mode_combo") or self._planner_mode_combo is None:
            return
        self._planner_mode = self._planner_mode_combo.currentData() or "adaptive"
        self._update_route_preview()
        self._update_scenario_detail()
        self._update_metric_summary()

    def _apply_selected_scenario(self):
        if not hasattr(self, "_scenario_selector") or self._scenario_selector is None:
            return
        self._apply_scenario_preset(self._scenario_selector.currentData())

    def _apply_scenario_preset(self, scenario_key):
        preset = get_scenario_preset(scenario_key)
        if not preset:
            return
        self._scenario_key = scenario_key
        for index in range(self._formation_type.count()):
            if self._formation_type.itemData(index) == preset["formation_type"]:
                self._formation_type.setCurrentIndex(index)
                break
        self._spacing.setValue(float(preset["spacing"]))
        self._heading.setValue(float(preset["heading_deg"]))
        anchor_x, anchor_y, anchor_z = preset["anchor"]
        self._anchor_x.setValue(float(anchor_x))
        self._anchor_y.setValue(float(anchor_y))
        self._anchor_z.setValue(float(anchor_z))
        self._anchor_xy = (float(anchor_x), float(anchor_y))
        self._safe_anchor_xy = self._anchor_xy
        self._task_points = [dict(point) for point in preset["task_points"]]
        self._scenario_layering_zone = dict(preset.get("layering_zone", {})) if preset.get("layering_zone") else None
        self._scenario_altitude_levels = dict(preset.get("altitude_levels", {}))
        self._scenario_altitude_assignments = {
            key: list(value) for key, value in dict(preset.get("altitude_assignments", {})).items()
        }
        self._scenario_comparison_routes = {
            key: [dict(point) for point in value]
            for key, value in dict(preset.get("comparison_routes", {})).items()
        }
        self._refresh_task_table()

        for obstacle_id in list(self._obstacle_order):
            obstacle = self._obstacles.get(obstacle_id)
            if obstacle is None:
                continue
            obstacle.enabled = False
            self._publish_obstacle_message(obstacle)
        self._obstacle_order = []
        for spec in preset.get("obstacles", []):
            self._publish_obstacle_message(self._build_obstacle_message_from_spec(spec))

        self._update_route_preview()
        self._update_scenario_detail()
        self._update_metric_summary()
        self._refresh_situation_view()
        self.action_result_signal.emit("实验场景已加载: {}".format(preset["label"]))

    def _compute_current_formation_error(self):
        if not self._vehicle_positions:
            self._live_vehicle_formation_errors = {}
            return 0.0
        active_vehicle_ids = formation_vehicle_ids(self._active_formation, VEHICLE_IDS)
        reference_anchor = self._safe_anchor_xy if self._safe_anchor_xy != (0.0, 0.0) else self._anchor_xy
        expected_positions = compute_formation_positions(
            self._active_formation,
            self._active_spacing,
            self._active_heading,
            reference_anchor,
            active_vehicle_ids,
        )
        errors = []
        per_vehicle_errors = {}
        for vehicle_id in active_vehicle_ids:
            actual = self._vehicle_positions.get(vehicle_id)
            if actual is None:
                continue
            expected_xy = expected_positions.get(vehicle_id)
            if expected_xy is None:
                continue
            error_value = math.hypot(actual[0] - expected_xy[0], actual[1] - expected_xy[1])
            per_vehicle_errors[vehicle_id] = error_value
            errors.append(error_value)
        self._live_vehicle_formation_errors = per_vehicle_errors
        if not errors:
            return 0.0
        return sum(errors) / float(len(errors))

    def _compute_current_obstacle_clearance(self):
        if not self._vehicle_positions:
            return float("inf")
        active_vehicle_ids = formation_vehicle_ids(self._active_formation, VEHICLE_IDS)
        position_points = [
            {"x": value[0], "y": value[1], "z": value[2]}
            for vehicle_id, value in self._vehicle_positions.items()
            if vehicle_id in active_vehicle_ids
        ]
        if not position_points:
            return float("inf")
        return min_clearance_to_obstacles(
            position_points,
            self._active_obstacle_specs(),
            extra_margin=self._vehicle_body_radius,
        )

    def _planned_route_formation_error(self, reference_points, actual_points):
        if not reference_points or not actual_points:
            return 0.0
        sample_count = max(1, min(len(reference_points), len(actual_points)))
        errors = []
        for index in range(sample_count):
            if sample_count == 1:
                ref_index = len(reference_points) - 1
                act_index = len(actual_points) - 1
            else:
                alpha = float(index) / float(sample_count - 1)
                ref_index = min(int(round(alpha * (len(reference_points) - 1))), len(reference_points) - 1)
                act_index = min(int(round(alpha * (len(actual_points) - 1))), len(actual_points) - 1)
            ref_point = reference_points[ref_index]
            act_point = actual_points[act_index]
            errors.append(
                math.sqrt(
                    (act_point["x"] - ref_point["x"]) ** 2
                    + (act_point["y"] - ref_point["y"]) ** 2
                    + (act_point["z"] - ref_point["z"]) ** 2
                )
            )
        return sum(errors) / float(len(errors))

    def _formation_error_threshold(self):
        active_formation = getattr(self, "_active_formation", self._formation_type.currentData())
        active_spacing = float(getattr(self, "_active_spacing", self._spacing.value()))
        if active_formation == "v_shape" and self._scenario_key == "layered_altitude_demo":
            return max(active_spacing * 0.22, 8.5)
        return max(active_spacing * 0.35, 12.0)

    def _refresh_live_metrics(self):
        self._formation_error_live = self._compute_current_formation_error()
        self._obstacle_clearance_live = self._compute_current_obstacle_clearance()
        if self._formation_switch_started_at is not None:
            if self._formation_error_live <= self._formation_error_threshold():
                self._formation_switch_successes += 1
                self._formation_switch_recovery_times.append(time.time() - self._formation_switch_started_at)
                self._formation_switch_started_at = None
                self._formation_switch_target = None
        self._update_metric_summary()

    def _latest_average_recovery_time(self):
        if not self._formation_switch_recovery_times:
            return 0.0
        return sum(self._formation_switch_recovery_times) / float(len(self._formation_switch_recovery_times))

    def _update_scenario_detail(self):
        if self._scenario_detail_label is None:
            return
        selected_key = self._scenario_key
        if hasattr(self, "_scenario_selector") and self._scenario_selector is not None:
            selected_key = self._scenario_selector.currentData() or selected_key
        preset = get_scenario_preset(selected_key) or {}
        scenario_name = preset.get("label", selected_key or "未选择")
        description = preset.get("description", "可加载典型场景并导出实验结果。")
        detail_lines = [
            "场景: {} | 对比模式: {}".format(scenario_name, planner_mode_label(self._planner_mode)),
            description,
        ]
        altitude_levels = preset.get("altitude_levels", {})
        altitude_assignments = preset.get("altitude_assignments", {})
        if altitude_levels and altitude_assignments:
            base_level = float(altitude_levels.get("base", preset.get("anchor", (0.0, 0.0, 0.0))[2]))
            for layer_key, title in [("upper", "上层"), ("base", "基准层"), ("lower", "下层")]:
                vehicle_ids = altitude_assignments.get(layer_key, [])
                if not vehicle_ids:
                    continue
                level = float(altitude_levels.get(layer_key, base_level))
                if abs(level - base_level) < 1e-6:
                    tag = "H"
                elif level > base_level:
                    tag = "H+{:.0f}m".format(level - base_level)
                else:
                    tag = "H{:.0f}m".format(level - base_level)
                detail_lines.append("{} {}: {}".format(title, tag, ", ".join(vehicle_ids)))
        self._scenario_detail_label.setText("\n".join(detail_lines))

    def _scenario_layer_tags(self):
        if not self._scenario_altitude_assignments or not self._scenario_altitude_levels:
            return {}
        base_level = float(self._scenario_altitude_levels.get("base", float(self._anchor_z.value())))
        tags = {}
        for layer_key, vehicle_ids in self._scenario_altitude_assignments.items():
            level = float(self._scenario_altitude_levels.get(layer_key, base_level))
            delta = level - base_level
            if abs(delta) < 1e-6:
                tag = "H"
            elif delta > 0:
                tag = "H+{:.0f}m".format(delta)
            else:
                tag = "H{:.0f}m".format(delta)
            for vehicle_id in vehicle_ids:
                tags[vehicle_id] = tag
        return tags

    def _update_metric_summary(self):
        if self._metric_label is None:
            return
        summary = self._latest_plan_summary or {}
        min_clearance = summary.get("min_clearance_m", float("inf"))
        clearance_text = "{:.1f} m".format(min_clearance) if math.isfinite(min_clearance) else "N/A"
        live_clearance_text = (
            "{:.1f} m".format(self._obstacle_clearance_live)
            if math.isfinite(self._obstacle_clearance_live)
            else "N/A"
        )
        switch_rate = (
            100.0 * float(self._formation_switch_successes) / float(self._formation_switch_attempts)
            if self._formation_switch_attempts
            else 0.0
        )
        self._metric_label.setText(
            "\n".join(
                [
                    "预计总航程: {0:.1f} m | 预计能耗: {1:.1f} Wh".format(
                        summary.get("avg_route_length_m", 0.0),
                        summary.get("avg_energy_wh", 0.0),
                    ),
                    "任务成功率: {0:.0f}% | 最小净空: {1} | 重规划次数: {2}".format(
                        summary.get("task_success_rate", 0.0) * 100.0,
                        clearance_text,
                        self._mission_refresh_count,
                    ),
                    "规划编队误差: {0:.1f} m | 编队保持能力: {1:.0f}%".format(
                        summary.get("avg_formation_error_m", 0.0),
                        max(
                            0.0,
                            100.0
                            * (
                                1.0
                                - (
                                    summary.get("avg_formation_error_m", 0.0)
                                    / max(self._active_spacing * 1.2, 1.0)
                                )
                            ),
                        ),
                    ),
                    "实时编队误差: {0:.1f} m | 实时障碍净空: {1}".format(
                        self._formation_error_live,
                        live_clearance_text,
                    ),
                    "队形恢复成功率: {0:.0f}% | 平均恢复时间: {1:.1f} s".format(
                        switch_rate,
                        self._latest_average_recovery_time(),
                    ),
                ]
            )
        )

    def _collect_experiment_snapshot(self):
        preset = get_scenario_preset(self._scenario_key) or {}
        summary = dict(self._latest_plan_summary or {})
        summary.update(
            {
                "scenario_key": self._scenario_key,
                "scenario_label": preset.get("label", self._scenario_key),
                "planner_mode": self._planner_mode,
                "planner_mode_label": planner_mode_label(self._planner_mode),
                "route_mode": self._route_mode,
                "mission_refresh_count": self._mission_refresh_count,
                "live_formation_error_m": self._formation_error_live,
                "live_obstacle_clearance_m": self._obstacle_clearance_live,
                "formation_switch_attempts": self._formation_switch_attempts,
                "formation_switch_successes": self._formation_switch_successes,
                "avg_recovery_time_s": self._latest_average_recovery_time(),
                "formation_keep_score": max(
                    0.0,
                    1.0 - (self._formation_error_live / max(self._active_spacing * 1.2, 1.0)),
                ),
                "layering_zone": dict(self._scenario_layering_zone or {}),
                "altitude_levels": dict(self._scenario_altitude_levels),
                "altitude_assignments": dict(self._scenario_altitude_assignments),
                "comparison_routes": dict(self._scenario_comparison_routes),
                "vehicle_routes": self._plan_vehicle_routes,
                "task_points": list(self._task_points),
                "obstacles": self._active_obstacle_specs(),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
        )
        return summary

    def _export_experiment_snapshot(self):
        snapshot = self._collect_experiment_snapshot()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        stem = "{}_{}".format(snapshot["scenario_key"], snapshot["planner_mode"])
        json_path = self._results_dir / "{}_{}.json".format(stem, timestamp)
        csv_path = self._results_dir / "experiment_summary.csv"
        screenshot_path = self._results_dir / "{}_{}.png".format(stem, timestamp)

        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(snapshot, handle, ensure_ascii=False, indent=2)

        row = {
            key: value
            for key, value in snapshot.items()
            if key not in {"vehicle_routes", "task_points", "obstacles", "timestamp"}
        }
        row["timestamp"] = snapshot["timestamp"]
        write_header = not csv_path.exists()
        csv_encoding = "utf-8-sig" if write_header else "utf-8"
        with csv_path.open("a", encoding=csv_encoding, newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        self.grab().save(str(screenshot_path))
        self._last_export_path = str(json_path)
        self.action_result_signal.emit("实验快照已导出: {}".format(json_path))
        self._update_metric_summary()

    def _publish_formation(self):
        was_active = self._mission_phase in {"armed", "mission_starting", "mission_active", "mission_refreshing"}
        msg = FormationCommand()
        msg.formation_type = self._formation_type.currentData()
        msg.spacing = float(self._spacing.value())
        msg.heading_deg = float(self._heading.value())
        msg.anchor = build_anchor_pose(float(self._anchor_x.value()), float(self._anchor_y.value()), float(self._anchor_z.value()))
        msg.vehicle_ids = formation_vehicle_ids(msg.formation_type, VEHICLE_IDS)
        self._active_formation = msg.formation_type or self._active_formation
        self._active_spacing = msg.spacing
        self._active_heading = msg.heading_deg
        self._formation_pub.publish(msg)
        self._mission_anchor_pub.publish(msg.anchor)
        self._anchor_xy = (msg.anchor.pose.position.x, msg.anchor.pose.position.y)
        if was_active:
            self._formation_switch_attempts += 1
            self._formation_switch_started_at = time.time()
            self._formation_switch_target = msg.formation_type
        if not was_active:
            self._publish_mission_phase("formation_ready")
        self._update_route_preview()
        self._refresh_situation_view()
        self._maybe_refresh_active_mission("编队变更")
        self._refresh_live_metrics()

    def _publish_obstacle(self):
        msg = Obstacle()
        self._obstacle_sequence += 1
        msg.id = "obstacle_{:02d}".format(self._obstacle_sequence)
        msg.shape = self._obs_shape.currentData()
        msg.enabled = True
        msg.pose.position.x = float(self._obs_x.value())
        msg.pose.position.y = float(self._obs_y.value())
        obstacle_height = float(self._obs_h.value())
        obstacle_bottom_z = float(self._obs_z.value())
        msg.pose.position.z = obstacle_bottom_z + obstacle_height / 2.0
        msg.pose.orientation.w = 1.0
        msg.size.x = float(self._obs_sx.value())
        msg.size.y = float(self._obs_sy.value())
        msg.size.z = obstacle_height
        self._publish_obstacle_message(msg)

    def _publish_obstacle_message(self, msg):
        self._obstacle_pub.publish(msg)
        self._obstacles[msg.id] = msg
        if msg.id not in self._obstacle_order:
            self._obstacle_order.append(msg.id)
        if not msg.enabled and msg.id in self._obstacle_order:
            self._obstacle_order = [obstacle_id for obstacle_id in self._obstacle_order if obstacle_id != msg.id]
        self._update_route_preview()
        self._maybe_refresh_active_mission("障碍物更新")
        self._refresh_live_metrics()

    def _remove_last_obstacle(self):
        if not self._obstacle_order:
            self.action_result_signal.emit("集群状态: 没有可删除的障碍物")
            return
        obstacle_id = self._obstacle_order[-1]
        obstacle = self._obstacles.get(obstacle_id)
        if obstacle is None:
            self._obstacle_order.pop()
            self._update_route_preview()
            return
        obstacle.enabled = False
        self._publish_obstacle_message(obstacle)
        self.action_result_signal.emit("集群状态: 已删除 {}".format(obstacle_id.replace("obstacle_", "障碍")))

    def _clear_obstacles(self):
        active_ids = list(self._obstacle_order)
        if not active_ids:
            self.action_result_signal.emit("集群状态: 没有可清空的障碍物")
            return
        for obstacle_id in active_ids:
            obstacle = self._obstacles.get(obstacle_id)
            if obstacle is None:
                continue
            obstacle.enabled = False
            self._obstacle_pub.publish(obstacle)
            self._obstacles[obstacle_id] = obstacle
        self._obstacle_order = []
        self._update_route_preview()
        self._maybe_refresh_active_mission("障碍物更新")
        self.action_result_signal.emit("集群状态: 已清空障碍物")

    def _generate_random_obstacles(self, count):
        self._generate_random_obstacles_with_mode(count, mode="scatter")

    def _generate_obstacle_field(self, mode):
        presets = {
            "sparse": 8,
            "dense": 18,
            "corridor": 12,
            "checkerboard": 16,
            "s_curve": 12,
        }
        self._generate_random_obstacles_with_mode(presets.get(mode, 10), mode=mode)

    def _obstacle_field_bounds(self, seed_points):
        field_center_x = float(self._obs_field_cx.value())
        field_center_y = float(self._obs_field_cy.value())
        field_width = max(float(self._obs_field_w.value()), 40.0)
        field_height = max(float(self._obs_field_hh.value()), 40.0)
        min_x = field_center_x - field_width / 2.0
        max_x = field_center_x + field_width / 2.0
        min_y = field_center_y - field_height / 2.0
        max_y = field_center_y + field_height / 2.0
        if seed_points:
            altitude_values = [point["z"] for point in seed_points]
            default_z = max(0.0, min(altitude_values) - 20.0)
        else:
            default_z = 0.0
        return min_x, max_x, min_y, max_y, default_z

    def _field_center(self):
        return float(self._obs_field_cx.value()), float(self._obs_field_cy.value())

    def _generate_random_obstacles_with_mode(self, count, mode="scatter"):
        if count <= 0:
            return
        seed_points = self._planned_task_path or self._requested_task_path or self._task_points
        min_x, max_x, min_y, max_y, default_z = self._obstacle_field_bounds(seed_points)
        field_center_x, field_center_y = self._field_center()
        if mode == "checkerboard":
            created = self._generate_checkerboard_field(seed_points, default_z, min_x, max_x, min_y, max_y)
            self.action_result_signal.emit("集群状态: 棋盘障碍场 已生成 {} 个障碍物".format(created))
            return
        if mode == "s_curve":
            created = self._generate_s_curve_field(seed_points, default_z, min_x, max_x, min_y, max_y)
            self.action_result_signal.emit("集群状态: S 型通道场 已生成 {} 个障碍物".format(created))
            return
        created = 0
        attempts = 0
        placed_obstacles = [
            obstacle
            for obstacle in self._obstacles.values()
            if obstacle.enabled
        ]
        while created < count and attempts < count * 20:
            attempts += 1
            if mode == "sparse":
                sx = random.choice([18.0, 22.0, 28.0, 35.0])
                sy = random.choice([18.0, 22.0, 28.0, 35.0])
            elif mode == "dense":
                sx = random.choice([22.0, 28.0, 35.0, 45.0, 60.0])
                sy = random.choice([22.0, 28.0, 35.0, 45.0, 60.0])
            elif mode == "corridor":
                sx = random.choice([24.0, 30.0, 36.0, 42.0])
                sy = random.choice([40.0, 50.0, 60.0, 70.0])
            else:
                sx = random.choice([18.0, 22.0, 28.0, 35.0, 45.0, 60.0])
                sy = random.choice([18.0, 22.0, 28.0, 35.0, 45.0, 60.0])
            if len(seed_points) >= 2:
                segment_index = random.randrange(len(seed_points) - 1)
                first_point = seed_points[segment_index]
                second_point = seed_points[segment_index + 1]
                alpha = random.uniform(0.1, 0.9)
                base_x = first_point["x"] + (second_point["x"] - first_point["x"]) * alpha
                base_y = first_point["y"] + (second_point["y"] - first_point["y"]) * alpha
                heading_rad = math.atan2(second_point["y"] - first_point["y"], second_point["x"] - first_point["x"])
                left_axis = (-math.sin(heading_rad), math.cos(heading_rad))
                forward_axis = (math.cos(heading_rad), math.sin(heading_rad))
                if mode == "sparse":
                    lateral_offset = random.choice([-1.0, 1.0]) * random.uniform(55.0, 160.0)
                    forward_offset = random.uniform(-45.0, 45.0)
                elif mode == "dense":
                    lateral_offset = random.choice([-1.0, 1.0]) * random.uniform(22.0, 95.0)
                    forward_offset = random.uniform(-18.0, 18.0)
                elif mode == "corridor":
                    gate_half_width = random.uniform(18.0, 34.0)
                    side_clearance = random.uniform(24.0, 52.0)
                    lateral_offset = random.choice([-1.0, 1.0]) * (gate_half_width + side_clearance + sy * 0.25)
                    forward_offset = random.uniform(-10.0, 10.0)
                else:
                    lateral_offset = random.choice([-1.0, 1.0]) * random.uniform(28.0, 110.0)
                    forward_offset = random.uniform(-25.0, 25.0)
                x_value = base_x + left_axis[0] * lateral_offset + forward_axis[0] * forward_offset
                y_value = base_y + left_axis[1] * lateral_offset + forward_axis[1] * forward_offset
                if not (min_x <= x_value <= max_x and min_y <= y_value <= max_y):
                    x_value = random.uniform(min_x, max_x)
                    y_value = random.uniform(min_y, max_y)
            else:
                x_value = random.uniform(min_x, max_x)
                y_value = random.uniform(min_y, max_y)
            x_value = min(max(x_value, min_x), max_x)
            y_value = min(max(y_value, min_y), max_y)
            candidate = Obstacle()
            self._obstacle_sequence += 1
            candidate.id = "obstacle_{:02d}".format(self._obstacle_sequence)
            candidate.shape = self._obs_shape.currentData()
            candidate.enabled = True
            candidate.pose.position.x = x_value
            candidate.pose.position.y = y_value
            candidate.pose.position.z = default_z + 60.0
            candidate.pose.orientation.w = 1.0
            candidate.size.x = sx
            candidate.size.y = sy
            candidate.size.z = 120.0
            too_close = False
            for obstacle in placed_obstacles:
                if mode == "sparse":
                    min_spacing = max(candidate.size.x, candidate.size.y, obstacle.size.x, obstacle.size.y) * 1.6 + 42.0
                elif mode == "dense":
                    min_spacing = max(candidate.size.x, candidate.size.y, obstacle.size.x, obstacle.size.y) * 1.0 + 16.0
                elif mode == "corridor":
                    min_spacing = max(candidate.size.x, candidate.size.y, obstacle.size.x, obstacle.size.y) * 1.05 + 24.0
                else:
                    min_spacing = max(candidate.size.x, candidate.size.y, obstacle.size.x, obstacle.size.y) * 1.15 + 18.0
                distance = math.hypot(
                    candidate.pose.position.x - obstacle.pose.position.x,
                    candidate.pose.position.y - obstacle.pose.position.y,
                )
                if distance < min_spacing:
                    too_close = True
                    break
            if too_close:
                continue
            if math.hypot(x_value - field_center_x, y_value - field_center_y) < max(sx, sy) * 0.6:
                continue
            self._publish_obstacle_message(candidate)
            placed_obstacles.append(candidate)
            created += 1
        label = {
            "sparse": "稀疏障碍场",
            "dense": "密集障碍场",
            "corridor": "通道型障碍场",
        }.get(mode, "随机障碍物")
        self.action_result_signal.emit("集群状态: {} 已生成 {} 个障碍物".format(label, created))

    def _create_obstacle_at(self, x_value, y_value, sx, sy, default_z, height=None):
        candidate = Obstacle()
        self._obstacle_sequence += 1
        candidate.id = "obstacle_{:02d}".format(self._obstacle_sequence)
        candidate.shape = self._obs_shape.currentData()
        candidate.enabled = True
        candidate.pose.position.x = x_value
        candidate.pose.position.y = y_value
        obstacle_height = 120.0 if height is None else height
        candidate.pose.position.z = default_z + obstacle_height / 2.0
        candidate.pose.orientation.w = 1.0
        candidate.size.x = sx
        candidate.size.y = sy
        candidate.size.z = obstacle_height
        return candidate

    def _generate_checkerboard_field(self, seed_points, default_z, min_x, max_x, min_y, max_y):
        if len(seed_points) >= 2:
            first_point = seed_points[0]
            last_point = seed_points[-1]
            heading_rad = math.atan2(last_point["y"] - first_point["y"], last_point["x"] - first_point["x"])
            forward_axis = (math.cos(heading_rad), math.sin(heading_rad))
            left_axis = (-math.sin(heading_rad), math.cos(heading_rad))
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0
            origin_x = center_x - forward_axis[0] * 95.0
            origin_y = center_y - forward_axis[1] * 95.0
        else:
            forward_axis = (1.0, 0.0)
            left_axis = (0.0, 1.0)
            origin_x, origin_y = (min_x + max_x) / 2.0 - 95.0, (min_y + max_y) / 2.0
        forward_steps = [0.0, 65.0, 130.0, 195.0]
        lateral_rows = [-95.0, -30.0, 30.0, 95.0]
        created = 0
        for row_index, lateral in enumerate(lateral_rows):
            for col_index, forward in enumerate(forward_steps):
                if (row_index + col_index) % 2 != 0:
                    continue
                x_value = origin_x + forward_axis[0] * forward + left_axis[0] * lateral
                y_value = origin_y + forward_axis[1] * forward + left_axis[1] * lateral
                if not (min_x <= x_value <= max_x and min_y <= y_value <= max_y):
                    continue
                candidate = self._create_obstacle_at(x_value, y_value, 26.0, 26.0, default_z, height=120.0)
                self._publish_obstacle_message(candidate)
                created += 1
        return created

    def _generate_s_curve_field(self, seed_points, default_z, min_x, max_x, min_y, max_y):
        if len(seed_points) >= 2:
            first_point = seed_points[0]
            last_point = seed_points[-1]
            heading_rad = math.atan2(last_point["y"] - first_point["y"], last_point["x"] - first_point["x"])
            forward_axis = (math.cos(heading_rad), math.sin(heading_rad))
            left_axis = (-math.sin(heading_rad), math.cos(heading_rad))
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0
            origin_x = center_x - forward_axis[0] * 110.0
            origin_y = center_y - forward_axis[1] * 40.0
        else:
            forward_axis = (1.0, 0.0)
            left_axis = (0.0, 1.0)
            origin_x, origin_y = (min_x + max_x) / 2.0 - 110.0, (min_y + max_y) / 2.0
        created = 0
        pattern = [
            (30.0, 65.0),
            (30.0, -65.0),
            (95.0, 65.0),
            (95.0, -65.0),
            (160.0, -65.0),
            (160.0, 65.0),
            (225.0, -65.0),
            (225.0, 65.0),
        ]
        for forward, lateral in pattern:
            x_value = origin_x + forward_axis[0] * forward + left_axis[0] * lateral
            y_value = origin_y + forward_axis[1] * forward + left_axis[1] * lateral
            if not (min_x <= x_value <= max_x and min_y <= y_value <= max_y):
                continue
            candidate = self._create_obstacle_at(x_value, y_value, 30.0, 54.0, default_z, height=120.0)
            self._publish_obstacle_message(candidate)
            created += 1
        return created

    def _publish_anchor(self):
        msg = build_anchor_pose(float(self._anchor_x.value()), float(self._anchor_y.value()), float(self._anchor_z.value()))
        self._mission_anchor_pub.publish(msg)
        self._anchor_xy = (msg.pose.position.x, msg.pose.position.y)
        self._update_route_preview()
        self._refresh_situation_view()

    def _toggle_field_selection_mode(self, enabled):
        if self._situation_view is not None:
            self._situation_view.set_field_selection_mode(enabled)
        if enabled:
            self.action_result_signal.emit("集群状态: 请在右侧画布左键拖拽框选障碍生成区域")
        else:
            self.action_result_signal.emit("集群状态: 已退出障碍区域框选")

    def _set_obstacle_field_region(self, center_x, center_y, width, height):
        self._obs_field_cx.setValue(center_x)
        self._obs_field_cy.setValue(center_y)
        self._obs_field_w.setValue(width)
        self._obs_field_hh.setValue(height)
        if self._field_select_btn is not None and self._field_select_btn.isChecked():
            self._field_select_btn.blockSignals(True)
            self._field_select_btn.setChecked(False)
            self._field_select_btn.blockSignals(False)
        if self._situation_view is not None:
            self._situation_view.set_field_selection_mode(False)
        self._refresh_situation_view()
        self.action_result_signal.emit(
            "集群状态: 已设置障碍区域 中心({:.1f}, {:.1f}) 范围({:.1f}, {:.1f})".format(
                center_x, center_y, width, height
            )
        )

    def _center_on_swarm(self):
        if not self._vehicle_positions:
            return
        x_value = sum(position[0] for position in self._vehicle_positions.values()) / len(self._vehicle_positions)
        y_value = sum(position[1] for position in self._vehicle_positions.values()) / len(self._vehicle_positions)
        self._anchor_x.setValue(x_value)
        self._anchor_y.setValue(y_value)
        self._publish_anchor()

    def _set_anchor_from_canvas(self, x_value, y_value):
        self._anchor_x.setValue(x_value)
        self._anchor_y.setValue(y_value)
        self._publish_anchor()

    def _reset_canvas_view(self):
        if self._situation_view is not None:
            self._situation_view.reset_view()

    def _call_arm_service(self, arm_value):
        if arm_value:
            self._publish_mission_phase("arming")
        threading.Thread(target=self._call_arm_service_worker, args=(arm_value,), daemon=True).start()

    def _start_mission(self):
        self._publish_mission_phase("mission_starting")
        threading.Thread(target=self._start_mission_worker, daemon=True).start()

    def _upload_formation_mission(self):
        threading.Thread(target=self._upload_formation_mission_worker, daemon=True).start()

    def _maybe_refresh_active_mission(self, trigger_label):
        if self._mission_phase not in {"mission_uploaded", "armed", "mission_starting", "mission_active"}:
            return
        self._mission_refresh_reason = trigger_label
        self._last_refresh_request_time = time.time()
        if not self._mission_refresh_lock.acquire(blocking=False):
            self._mission_refresh_pending = True
            return
        threading.Thread(
            target=self._refresh_active_mission_worker,
            args=(trigger_label,),
            daemon=True,
        ).start()

    def _refresh_active_mission_worker(self, trigger_label):
        try:
            rospy.sleep(0.8)
            latest_reason = self._mission_refresh_reason or trigger_label
            if time.time() - self._last_refresh_request_time < 0.6:
                rospy.sleep(0.6)
            self._mission_refresh_count += 1
            if self._mission_phase == "mission_uploaded":
                self.action_result_signal.emit("集群状态: {}，正在更新已上传任务".format(latest_reason))
                self._upload_formation_mission_worker()
            else:
                self._publish_mission_phase("mission_refreshing")
                self.action_result_signal.emit("集群状态: {}，正在平滑刷新飞行任务".format(latest_reason))
                self._set_mode_for_refresh("AUTO.LOITER")
                rospy.sleep(0.6)
                self._upload_formation_mission_worker()
                self._start_mission_worker()
        finally:
            self._mission_refresh_lock.release()
            if self._mission_refresh_pending:
                self._mission_refresh_pending = False
                self._maybe_refresh_active_mission(self._mission_refresh_reason or "任务刷新排队执行")

    def _upload_formation_mission_worker(self):
        refreshing_active_mission = self._mission_phase in {"armed", "mission_starting", "mission_active", "mission_refreshing"}
        formation_type = self._formation_type.currentData()
        spacing = float(self._spacing.value())
        heading_deg = float(self._heading.value())
        anchor_x = float(self._anchor_x.value())
        anchor_y = float(self._anchor_y.value())
        altitude = float(self._anchor_z.value())
        positions = compute_formation_positions(
            formation_type,
            spacing,
            heading_deg,
            (anchor_x, anchor_y),
            formation_vehicle_ids(formation_type, VEHICLE_IDS),
        )
        heading_rad = math.radians(heading_deg)
        landing_forward = (math.cos(heading_rad), math.sin(heading_rad))
        safe_delta_x = self._safe_anchor_xy[0] - anchor_x
        safe_delta_y = self._safe_anchor_xy[1] - anchor_y
        mission_points = self._task_points or [
            {
                "x": anchor_x,
                "y": anchor_y,
                "z": altitude,
            }
        ]
        adjusted_mission_points = [
            {
                "x": task_point["x"] + safe_delta_x,
                "y": task_point["y"] + safe_delta_y,
                "z": task_point["z"],
            }
            for task_point in mission_points
        ]
        adjusted_mission_points = self._prepare_fixed_wing_mission_points(
            adjusted_mission_points,
            formation_type,
            spacing,
            heading_deg,
        )
        self._formation_stage_hold_distance = 0.0
        self._formation_stage_release_distance = 0.0
        center_route_points = self._plan_center_route_preserving_formation(adjusted_mission_points, formation_type, spacing)
        if self._mission_phase in {"armed", "mission_starting", "mission_active", "mission_refreshing"} and self._vehicle_positions:
            swarm_center_x = sum(value[0] for value in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
            swarm_center_y = sum(value[1] for value in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
            swarm_center_z = sum(value[2] for value in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
            center_route_points = self._trim_route_from_current_progress(
                center_route_points,
                (swarm_center_x, swarm_center_y),
                swarm_center_z,
            )
            center_route_points = self._prepend_transition_regroup_point(center_route_points, formation_type, spacing)
        elif self._vehicle_positions:
            center_route_points = self._prepend_launch_staging_points(center_route_points, formation_type, spacing)
        self._requested_task_path = [dict(point) for point in adjusted_mission_points]
        self._planned_task_path = [dict(point) for point in center_route_points]
        active_vehicle_ids = formation_vehicle_ids(formation_type, VEHICLE_IDS)
        standby_ids = standby_vehicle_ids(formation_type, VEHICLE_IDS)
        result = True
        details = []
        self._mission_counts = {}
        self._plan_vehicle_routes = {}
        route_lengths = []
        route_energies = []
        route_clearances = []
        route_formation_errors = []
        obstacle_specs = self._active_obstacle_specs()
        for vehicle_id in active_vehicle_ids:
            fix = self._global_positions.get(vehicle_id)
            local_pose = self._vehicle_positions.get(vehicle_id)
            if fix is None or local_pose is None:
                result = False
                details.append("{}缺少定位".format(vehicle_id))
                continue
            vehicle_route_targets = self._offset_route_for_vehicle(center_route_points, formation_type, spacing, vehicle_id)
            vehicle_route_with_start = [
                {"x": local_pose[0], "y": local_pose[1], "z": max(vehicle_route_targets[0]["z"], altitude)}
            ] + vehicle_route_targets
            preserve_center_shape = (
                self._scenario_key == "layered_altitude_demo"
                and formation_type == "v_shape"
            )
            if preserve_center_shape or self._route_mode in {"corridor_compressing", "single_file_regrouping"}:
                refined_vehicle_points = vehicle_route_with_start
            else:
                refined_vehicle_points = self._plan_task_points_with_avoidance(vehicle_route_with_start)
            if len(refined_vehicle_points) > len(vehicle_route_with_start):
                self._route_mode = "vehicle_deformed"
            self._plan_vehicle_routes[vehicle_id] = [dict(point) for point in refined_vehicle_points]
            route_lengths.append(polyline_length(refined_vehicle_points))
            route_energies.append(estimate_route_energy_wh(refined_vehicle_points))
            route_clearances.append(
                min_clearance_to_obstacles(
                    refined_vehicle_points,
                    obstacle_specs,
                    extra_margin=self._vehicle_body_radius,
                )
            )
            route_formation_errors.append(
                self._planned_route_formation_error(vehicle_route_with_start, refined_vehicle_points)
            )
            vehicle_mission_points = self._compress_route_points(refined_vehicle_points[1:], max_points=16)
            if not vehicle_mission_points:
                result = False
                details.append("{} empty route".format(vehicle_id))
                continue
            mission_items = self._build_mission_items(
                vehicle_route_with_start,
                fix,
                altitude,
                landing_forward,
                include_takeoff=not refreshing_active_mission,
            )
            clear_service_name = "/{}/mavros/mission/clear".format(vehicle_id)
            push_service_name = "/{}/mavros/mission/push".format(vehicle_id)
            try:
                if vehicle_id not in self._waypoint_clear_clients:
                    rospy.wait_for_service(clear_service_name, timeout=1.0)
                    self._waypoint_clear_clients[vehicle_id] = rospy.ServiceProxy(clear_service_name, WaypointClear)
                if vehicle_id not in self._waypoint_push_clients:
                    rospy.wait_for_service(push_service_name, timeout=1.0)
                    self._waypoint_push_clients[vehicle_id] = rospy.ServiceProxy(push_service_name, WaypointPush)
                ok = False
                response = None
                for _attempt in range(3):
                    try:
                        self._waypoint_clear_clients[vehicle_id]()
                        rospy.sleep(0.15)
                        response = self._waypoint_push_clients[vehicle_id](0, mission_items)
                        ok = bool(response.success) and response.wp_transfered == len(mission_items)
                        if ok:
                            break
                    except (rospy.ROSException, rospy.ServiceException) as exc:
                        rospy.logwarn(
                            "任务上传重试 %s attempt=%d clear/push failed: %s",
                            vehicle_id,
                            _attempt + 1,
                            exc,
                        )
                    rospy.sleep(0.35)
                result = result and ok
                if ok:
                    self._mission_counts[vehicle_id] = len(mission_items)
                if not ok:
                    details.append("{}任务上传失败".format(vehicle_id))
            except (rospy.ROSException, rospy.ServiceException) as exc:
                result = False
                details.append("{}任务异常".format(vehicle_id))
                rospy.logwarn("任务上传失败 %s: %s", vehicle_id, exc)
        for vehicle_id in standby_ids:
            local_pose = self._vehicle_positions.get(vehicle_id)
            fix = self._global_positions.get(vehicle_id)
            if fix is None or local_pose is None or not center_route_points:
                continue
            standby_targets = [dict(point) for point in center_route_points]
            standby_route_with_start = [
                {"x": local_pose[0], "y": local_pose[1], "z": max(standby_targets[0]["z"], altitude)}
            ] + standby_targets
            try:
                mission_items = self._build_mission_items(
                    standby_route_with_start,
                    fix,
                    altitude,
                    landing_forward,
                    include_takeoff=not refreshing_active_mission,
                )
                clear_service_name = "/{}/mavros/mission/clear".format(vehicle_id)
                push_service_name = "/{}/mavros/mission/push".format(vehicle_id)
                if vehicle_id not in self._waypoint_clear_clients:
                    rospy.wait_for_service(clear_service_name, timeout=1.0)
                    self._waypoint_clear_clients[vehicle_id] = rospy.ServiceProxy(clear_service_name, WaypointClear)
                if vehicle_id not in self._waypoint_push_clients:
                    rospy.wait_for_service(push_service_name, timeout=1.0)
                    self._waypoint_push_clients[vehicle_id] = rospy.ServiceProxy(push_service_name, WaypointPush)
                self._waypoint_clear_clients[vehicle_id]()
                rospy.sleep(0.1)
                response = self._waypoint_push_clients[vehicle_id](0, mission_items)
                if bool(response.success):
                    self._mission_counts[vehicle_id] = len(mission_items)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("备用机任务上传失败 %s: %s", vehicle_id, exc)
        self._latest_plan_summary = {
            "avg_route_length_m": sum(route_lengths) / float(len(route_lengths)) if route_lengths else 0.0,
            "avg_energy_wh": sum(route_energies) / float(len(route_energies)) if route_energies else 0.0,
            "min_clearance_m": min(route_clearances) if route_clearances else float("inf"),
            "avg_formation_error_m": (
                sum(route_formation_errors) / float(len(route_formation_errors))
                if route_formation_errors
                else 0.0
            ),
            "task_success_rate": (
                float(len([vehicle_id for vehicle_id in active_vehicle_ids if vehicle_id in self._mission_counts])) / float(len(active_vehicle_ids))
                if active_vehicle_ids
                else 0.0
            ),
        }
        suffix = "成功" if result else "部分失败"
        if details:
            suffix += " ({})".format(",".join(details[:3]))
        self.action_result_signal.emit("集群状态: 上传编队任务 {}".format(suffix))
        self._update_metric_summary()

        if result and not refreshing_active_mission:
            self._publish_mission_phase("mission_uploaded")

    def _start_mission_worker(self):
        mode_ok = True
        start_ok = True
        details = []
        for vehicle_id in formation_vehicle_ids(self._active_formation, VEHICLE_IDS):
            mode_service_name = "/{}/mavros/set_mode".format(vehicle_id)
            command_service_name = "/{}/mavros/cmd/command".format(vehicle_id)
            try:
                if vehicle_id not in self._mode_clients:
                    rospy.wait_for_service(mode_service_name, timeout=1.0)
                    self._mode_clients[vehicle_id] = rospy.ServiceProxy(mode_service_name, SetMode)
                if vehicle_id not in self._command_clients:
                    rospy.wait_for_service(command_service_name, timeout=1.0)
                    self._command_clients[vehicle_id] = rospy.ServiceProxy(command_service_name, CommandLong)

                mode_response = self._mode_clients[vehicle_id](0, "AUTO.MISSION")
                mode_ok = mode_ok and bool(mode_response.mode_sent)
                if not mode_response.mode_sent:
                    details.append("{}任务模式失败".format(vehicle_id))
                    continue

                last_item = max(self._mission_counts.get(vehicle_id, 1) - 1, 0)
                command_response = self._command_clients[vehicle_id](
                    False,
                    300,
                    0,
                    0.0,
                    float(last_item),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
                ok = bool(command_response.success)
                start_ok = start_ok and ok
                if not ok:
                    details.append("{}开始任务失败(result={})".format(vehicle_id, command_response.result))
            except (rospy.ROSException, rospy.ServiceException) as exc:
                mode_ok = False
                start_ok = False
                details.append("{}开始任务异常".format(vehicle_id))
                rospy.logwarn("开始任务失败 %s: %s", vehicle_id, exc)

        if mode_ok and start_ok:
            self.action_result_signal.emit("集群状态: 开始任务 已下发")
        else:
            suffix = " ({})".format(",".join(details[:3])) if details else ""
            self.action_result_signal.emit("集群状态: 开始任务 部分失败{}".format(suffix))

        if mode_ok and start_ok:
            self._publish_mission_phase("mission_active")

    def _set_mode_for_refresh(self, custom_mode):
        result = True
        for vehicle_id in formation_vehicle_ids(self._active_formation, VEHICLE_IDS):
            service_name = "/{}/mavros/set_mode".format(vehicle_id)
            try:
                if vehicle_id not in self._mode_clients:
                    rospy.wait_for_service(service_name, timeout=1.0)
                    self._mode_clients[vehicle_id] = rospy.ServiceProxy(service_name, SetMode)
                response = self._mode_clients[vehicle_id](0, custom_mode)
                result = result and bool(response.mode_sent)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                result = False
                rospy.logwarn("任务刷新前模式切换失败 %s -> %s: %s", vehicle_id, custom_mode, exc)
        if result and custom_mode == "AUTO.LOITER":
            self._publish_mission_phase("holding")
        return result

    def _add_task_point(self):
        self._task_points.append(
            {
                "x": float(self._task_x.value()),
                "y": float(self._task_y.value()),
                "z": float(self._task_alt.value()),
            }
        )
        if len(self._task_points) == 1:
            first_point = self._task_points[0]
            self._anchor_x.setValue(first_point["x"])
            self._anchor_y.setValue(first_point["y"])
            self._anchor_z.setValue(first_point["z"])
            self._anchor_xy = (first_point["x"], first_point["y"])
            self._safe_anchor_xy = self._anchor_xy
            self._refresh_situation_view()
        self._refresh_task_table()
        self._update_route_preview()

    def _remove_last_task_point(self):
        if self._task_points:
            self._task_points.pop()
            self._refresh_task_table()
            self._update_route_preview()

    def _clear_task_points(self):
        self._task_points = []
        self._refresh_task_table()
        self._update_route_preview()

    def _refresh_task_table(self):
        self._task_table.setRowCount(len(self._task_points))
        for row, task_point in enumerate(self._task_points):
            self._task_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self._task_table.setItem(row, 1, QTableWidgetItem("{:.1f}".format(task_point["x"])))
            self._task_table.setItem(row, 2, QTableWidgetItem("{:.1f}".format(task_point["y"])))
            self._task_table.setItem(row, 3, QTableWidgetItem("{:.1f}".format(task_point["z"])))
        self._update_workflow_status()

    def _publish_mission_phase(self, phase):
        self._mission_phase = phase
        self._mission_phase_pub.publish(String(data=phase))
        self._update_workflow_status()

    def _update_workflow_status(self):
        phase_map = {
            "idle": "待配置",
            "formation_ready": "编队已配置",
            "mission_uploaded": "任务已上传",
            "arming": "正在解锁",
            "armed": "已解锁待执行",
            "mission_starting": "正在开始任务",
            "mission_active": "任务执行中",
            "holding": "盘旋待命",
            "mission_refreshing": "正在平滑更新任务",
        }
        phase_text = phase_map.get(self._mission_phase, self._mission_phase or "待配置")
        task_count = len(self._task_points)
        upload_text = "已生成" if self._mission_counts else "未上传"
        detail_lines = [
            "当前阶段: {}".format(phase_text),
            "共同航点数量: {} 个".format(task_count),
            "任务上传: {}".format(upload_text),
            "障碍物数量: {} 个".format(len([obs for obs in self._obstacles.values() if obs.enabled])),
            "重组点数量: {} 个".format(len(self._regroup_points)),
            "参考点用途: 仅用于编队预览与手动对齐；首个共同航点会自动同步到这里",
            "航路模式: {}".format(
                {
                    "direct": "直连",
                    "formation_preserving": "保持编队",
                    "corridor_compressing": "通道压缩",
                    "single_file_regrouping": "单列穿越重组",
                    "vehicle_deformed": "局部变形",
                }.get(self._route_mode, self._route_mode)
            ),
        ]
        self._workflow_label.setText("任务流程: {}".format(phase_text))
        if self._workflow_detail_label is not None:
            self._workflow_detail_label.setText("\n".join(detail_lines))

    def _obstacle_affects_altitude(self, obstacle, start_z, end_z, vertical_margin=10.0):
        obstacle_bottom = obstacle.pose.position.z - obstacle.size.z / 2.0 - vertical_margin
        obstacle_top = obstacle.pose.position.z + obstacle.size.z / 2.0 + vertical_margin
        path_bottom = min(start_z, end_z)
        path_top = max(start_z, end_z)
        return not (path_top < obstacle_bottom or path_bottom > obstacle_top)

    def _flight_envelope_margin(self, base_margin):
        return base_margin + self._vehicle_body_radius + self._vehicle_tracking_margin + self._vehicle_turn_buffer

    def _segment_hits_obstacle(self, start_xy, end_xy, obstacle, margin, start_z=None, end_z=None):
        if start_z is not None and end_z is not None and not self._obstacle_affects_altitude(obstacle, start_z, end_z):
            return False
        inflated_margin = self._flight_envelope_margin(margin)
        min_x = obstacle.pose.position.x - obstacle.size.x / 2.0 - inflated_margin
        max_x = obstacle.pose.position.x + obstacle.size.x / 2.0 + inflated_margin
        min_y = obstacle.pose.position.y - obstacle.size.y / 2.0 - inflated_margin
        max_y = obstacle.pose.position.y + obstacle.size.y / 2.0 + inflated_margin
        x1, y1 = start_xy
        x2, y2 = end_xy
        dx = x2 - x1
        dy = y2 - y1
        p = [-dx, dx, -dy, dy]
        q = [x1 - min_x, max_x - x1, y1 - min_y, max_y - y1]
        u1 = 0.0
        u2 = 1.0
        for p_value, q_value in zip(p, q):
            if abs(p_value) < 1e-6:
                if q_value < 0.0:
                    return False
                continue
            t_value = q_value / p_value
            if p_value < 0.0:
                u1 = max(u1, t_value)
            else:
                u2 = min(u2, t_value)
            if u1 > u2:
                return False
        return True

    def _build_detour_points(self, start_xy, end_xy, obstacle, margin, start_z=None, end_z=None):
        inflated_margin = self._flight_envelope_margin(margin)
        min_x = obstacle.pose.position.x - obstacle.size.x / 2.0 - inflated_margin
        max_x = obstacle.pose.position.x + obstacle.size.x / 2.0 + inflated_margin
        min_y = obstacle.pose.position.y - obstacle.size.y / 2.0 - inflated_margin
        max_y = obstacle.pose.position.y + obstacle.size.y / 2.0 + inflated_margin
        corner_margin = max(inflated_margin * 0.65, 18.0)
        corner_min_x = min_x - corner_margin
        corner_max_x = max_x + corner_margin
        corner_min_y = min_y - corner_margin
        corner_max_y = max_y + corner_margin
        candidates = [
            [(start_xy[0], max_y), (end_xy[0], max_y)],
            [(start_xy[0], min_y), (end_xy[0], min_y)],
            [(min_x, start_xy[1]), (min_x, end_xy[1])],
            [(max_x, start_xy[1]), (max_x, end_xy[1])],
            [(corner_min_x, corner_min_y), (corner_min_x, corner_max_y), (corner_max_x, corner_max_y)],
            [(corner_max_x, corner_min_y), (corner_max_x, corner_max_y), (corner_min_x, corner_max_y)],
            [(corner_min_x, corner_max_y), (corner_max_x, corner_max_y), (corner_max_x, corner_min_y)],
            [(corner_max_x, corner_min_y), (corner_min_x, corner_min_y), (corner_min_x, corner_max_y)],
            [(corner_min_x, corner_min_y), (corner_max_x, corner_min_y)],
            [(corner_min_x, corner_max_y), (corner_max_x, corner_max_y)],
            [(corner_min_x, corner_min_y), (corner_min_x, corner_max_y)],
            [(corner_max_x, corner_min_y), (corner_max_x, corner_max_y)],
        ]
        best_path = None
        best_cost = None
        for candidate in candidates:
            points = [start_xy] + candidate + [end_xy]
            valid = True
            for first, second in zip(points[:-1], points[1:]):
                if self._segment_hits_obstacle(first, second, obstacle, 0.0, start_z, end_z):
                    valid = False
                    break
            if not valid:
                continue
            cost = 0.0
            for first, second in zip(points[:-1], points[1:]):
                cost += math.hypot(second[0] - first[0], second[1] - first[1])
            turns = max(0, len(candidate) - 1)
            cost += turns * max(margin * 0.4, 12.0)
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_path = candidate
        return best_path or []

    def _push_point_out_of_obstacles(self, point_xy, margin, altitude=None):
        adjusted_x, adjusted_y = point_xy
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if altitude is not None and not self._obstacle_affects_altitude(obstacle, altitude, altitude):
                continue
            inflated_margin = self._flight_envelope_margin(margin)
            half_x = obstacle.size.x / 2.0 + inflated_margin
            half_y = obstacle.size.y / 2.0 + inflated_margin
            delta_x = adjusted_x - obstacle.pose.position.x
            delta_y = adjusted_y - obstacle.pose.position.y
            if abs(delta_x) < half_x and abs(delta_y) < half_y:
                clearance_x = half_x - abs(delta_x)
                clearance_y = half_y - abs(delta_y)
                if clearance_x <= clearance_y:
                    adjusted_x += math.copysign(clearance_x + max(self._vehicle_body_radius, 6.0), delta_x if delta_x != 0.0 else 1.0)
                else:
                    adjusted_y += math.copysign(clearance_y + max(self._vehicle_body_radius, 6.0), delta_y if delta_y != 0.0 else 1.0)
        return adjusted_x, adjusted_y

    def _formation_envelope_margin(self, formation_type, spacing):
        offsets = compute_formation_offsets(formation_type, spacing, formation_vehicle_ids(formation_type, VEHICLE_IDS))
        lateral_span = 0.0
        longitudinal_span = 0.0
        for forward_offset, left_offset in offsets:
            lateral_span = max(lateral_span, abs(left_offset))
            longitudinal_span = max(longitudinal_span, abs(forward_offset))
        return max(lateral_span + spacing * 1.05, longitudinal_span * 0.45 + 48.0, 55.0)

    def _center_route_margin_for_scenario(self, formation_type, spacing):
        margin = self._formation_envelope_margin(formation_type, spacing)
        if self._scenario_key != "corridor_passage":
            return margin
        # In corridor demos we let the centerline probe the middle gap using
        # near-single-aircraft clearance, then rely on the downstream corridor
        # compression/single-file logic to shrink the formation through it.
        return min(margin, max(self._vehicle_body_radius * 0.25, 2.0))

    def _segment_progress(self, start_xy, end_xy, point_xy):
        seg_x = end_xy[0] - start_xy[0]
        seg_y = end_xy[1] - start_xy[1]
        denom = seg_x * seg_x + seg_y * seg_y
        if denom < 1e-6:
            return 0.0
        rel_x = point_xy[0] - start_xy[0]
        rel_y = point_xy[1] - start_xy[1]
        progress = (rel_x * seg_x + rel_y * seg_y) / denom
        return max(0.0, min(1.0, progress))

    def _project_point_to_segment(self, point_xy, start_xy, end_xy):
        seg_x = end_xy[0] - start_xy[0]
        seg_y = end_xy[1] - start_xy[1]
        denom = seg_x * seg_x + seg_y * seg_y
        if denom < 1e-6:
            return {"x": start_xy[0], "y": start_xy[1], "alpha": 0.0}
        rel_x = point_xy[0] - start_xy[0]
        rel_y = point_xy[1] - start_xy[1]
        alpha = max(0.0, min(1.0, (rel_x * seg_x + rel_y * seg_y) / denom))
        return {
            "x": start_xy[0] + seg_x * alpha,
            "y": start_xy[1] + seg_y * alpha,
            "alpha": alpha,
        }

    def _route_distance_until(self, route_points, segment_index, alpha):
        distance = 0.0
        for first, second in zip(route_points[:segment_index], route_points[1:segment_index + 1]):
            distance += math.hypot(second["x"] - first["x"], second["y"] - first["y"])
        if segment_index < len(route_points) - 1:
            first = route_points[segment_index]
            second = route_points[segment_index + 1]
            distance += math.hypot(second["x"] - first["x"], second["y"] - first["y"]) * alpha
        return distance

    def _trim_route_from_current_progress(self, route_points, current_xy, current_altitude):
        if len(route_points) < 2:
            return route_points
        candidates = []
        for index, (first, second) in enumerate(zip(route_points[:-1], route_points[1:])):
            projection = self._project_point_to_segment(current_xy, (first["x"], first["y"]), (second["x"], second["y"]))
            distance = math.hypot(current_xy[0] - projection["x"], current_xy[1] - projection["y"])
            along = self._route_distance_until(route_points, index, projection["alpha"])
            candidates.append(
                {
                    "distance": distance,
                    "along": along,
                    "index": index,
                    "projection": projection,
                }
            )
        if not candidates:
            return route_points

        min_distance = min(candidate["distance"] for candidate in candidates)
        distance_tolerance = max(20.0, min_distance * 0.35)
        eligible = [
            candidate
            for candidate in candidates
            if candidate["distance"] <= min_distance + distance_tolerance
        ]
        best = max(eligible, key=lambda candidate: (candidate["along"], -candidate["distance"]))
        segment_index = best["index"]
        projection = best["projection"]
        first = route_points[segment_index]
        second = route_points[segment_index + 1]
        projected_z = first["z"] + (second["z"] - first["z"]) * projection["alpha"]
        trimmed = [
            {
                "x": current_xy[0],
                "y": current_xy[1],
                "z": max(projected_z, current_altitude),
            }
        ]
        if math.hypot(current_xy[0] - projection["x"], current_xy[1] - projection["y"]) > 8.0:
            trimmed.append(
                {
                    "x": projection["x"],
                    "y": projection["y"],
                    "z": max(projected_z, current_altitude),
                }
            )
        trimmed.extend(dict(point) for point in route_points[segment_index + 1 :])
        return self._dedupe_route_points(trimmed, min_spacing=4.0)

    def _find_first_blocking_obstacle(self, start_xy, end_xy, margin, start_z, end_z):
        first_hit = None
        first_progress = None
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if not self._segment_hits_obstacle(start_xy, end_xy, obstacle, margin, start_z, end_z):
                continue
            obstacle_center = (obstacle.pose.position.x, obstacle.pose.position.y)
            progress = self._segment_progress(start_xy, end_xy, obstacle_center)
            if first_progress is None or progress < first_progress:
                first_progress = progress
                first_hit = obstacle
        return first_hit

    def _point_to_segment_distance(self, point_xy, start_xy, end_xy):
        seg_x = end_xy[0] - start_xy[0]
        seg_y = end_xy[1] - start_xy[1]
        seg_len_sq = seg_x * seg_x + seg_y * seg_y
        if seg_len_sq < 1e-6:
            return math.hypot(point_xy[0] - start_xy[0], point_xy[1] - start_xy[1])
        t_value = ((point_xy[0] - start_xy[0]) * seg_x + (point_xy[1] - start_xy[1]) * seg_y) / seg_len_sq
        t_value = max(0.0, min(1.0, t_value))
        proj_x = start_xy[0] + seg_x * t_value
        proj_y = start_xy[1] + seg_y * t_value
        return math.hypot(point_xy[0] - proj_x, point_xy[1] - proj_y)

    def _segment_near_obstacle_cluster(self, start_xy, end_xy, margin, altitude):
        near_count = 0
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if not self._obstacle_affects_altitude(obstacle, altitude, altitude):
                continue
            half_diag = math.hypot(obstacle.size.x / 2.0, obstacle.size.y / 2.0)
            proximity = self._point_to_segment_distance(
                (obstacle.pose.position.x, obstacle.pose.position.y),
                start_xy,
                end_xy,
            )
            if proximity <= half_diag + self._flight_envelope_margin(margin) * 1.15:
                near_count += 1
                if near_count >= 2:
                    return True
        return False

    def _point_hits_any_obstacle(self, point_xy, margin, altitude=None):
        x_value, y_value = point_xy
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if altitude is not None and not self._obstacle_affects_altitude(obstacle, altitude, altitude):
                continue
            inflated_margin = self._flight_envelope_margin(margin)
            half_x = obstacle.size.x / 2.0 + inflated_margin
            half_y = obstacle.size.y / 2.0 + inflated_margin
            if abs(x_value - obstacle.pose.position.x) <= half_x and abs(y_value - obstacle.pose.position.y) <= half_y:
                return True
        return False

    def _astar_segment_route(self, start_xy, end_xy, altitude, margin, cell_size, force_search=False):
        if self._point_hits_any_obstacle(start_xy, margin, altitude):
            start_xy = self._push_point_out_of_obstacles(start_xy, margin, altitude)
        if self._point_hits_any_obstacle(end_xy, margin, altitude):
            end_xy = self._push_point_out_of_obstacles(end_xy, margin, altitude)
        direct_collision = self._route_has_collisions(
            [{"x": start_xy[0], "y": start_xy[1], "z": altitude}, {"x": end_xy[0], "y": end_xy[1], "z": altitude}],
            margin,
        )
        near_cluster = self._segment_near_obstacle_cluster(start_xy, end_xy, margin, altitude)
        if not force_search and not direct_collision and not near_cluster:
            return [{"x": end_xy[0], "y": end_xy[1], "z": altitude}], False

        expanded = max(margin * 2.5, 120.0)
        min_x = min(start_xy[0], end_xy[0]) - expanded
        max_x = max(start_xy[0], end_xy[0]) + expanded
        min_y = min(start_xy[1], end_xy[1]) - expanded
        max_y = max(start_xy[1], end_xy[1]) + expanded
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if not self._obstacle_affects_altitude(obstacle, altitude, altitude):
                continue
            min_x = min(min_x, obstacle.pose.position.x - obstacle.size.x / 2.0 - expanded)
            max_x = max(max_x, obstacle.pose.position.x + obstacle.size.x / 2.0 + expanded)
            min_y = min(min_y, obstacle.pose.position.y - obstacle.size.y / 2.0 - expanded)
            max_y = max(max_y, obstacle.pose.position.y + obstacle.size.y / 2.0 + expanded)

        def point_to_cell(point_xy):
            return (
                int(round((point_xy[0] - min_x) / cell_size)),
                int(round((point_xy[1] - min_y) / cell_size)),
            )

        def cell_to_point(cell):
            return (
                min_x + cell[0] * cell_size,
                min_y + cell[1] * cell_size,
            )

        start_cell = point_to_cell(start_xy)
        goal_cell = point_to_cell(end_xy)
        x_cells = max(2, int(math.ceil((max_x - min_x) / cell_size)) + 1)
        y_cells = max(2, int(math.ceil((max_y - min_y) / cell_size)) + 1)

        blocked = set()
        for ix in range(x_cells):
            for iy in range(y_cells):
                world_xy = cell_to_point((ix, iy))
                if self._point_hits_any_obstacle(world_xy, margin, altitude):
                    blocked.add((ix, iy))
        blocked.discard(start_cell)
        blocked.discard(goal_cell)

        frontier = []
        heapq.heappush(frontier, (0.0, start_cell))
        came_from = {start_cell: None}
        cost_so_far = {start_cell: 0.0}
        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while frontier:
            _priority, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if nxt[0] < 0 or nxt[0] >= x_cells or nxt[1] < 0 or nxt[1] >= y_cells:
                    continue
                if nxt in blocked:
                    continue
                if dx != 0 and dy != 0:
                    if (current[0] + dx, current[1]) in blocked or (current[0], current[1] + dy) in blocked:
                        continue
                step_cost = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    heuristic = math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                    heapq.heappush(frontier, (new_cost + heuristic, nxt))
                    came_from[nxt] = current

        if goal_cell not in came_from:
            return [{"x": end_xy[0], "y": end_xy[1], "z": altitude}], False

        path_cells = []
        current = goal_cell
        while current is not None:
            path_cells.append(current)
            current = came_from[current]
        path_cells.reverse()

        route_points = []
        for index, cell in enumerate(path_cells[1:], start=1):
            if index == len(path_cells) - 1:
                route_points.append({"x": end_xy[0], "y": end_xy[1], "z": altitude})
            else:
                waypoint_xy = cell_to_point(cell)
                route_points.append({"x": waypoint_xy[0], "y": waypoint_xy[1], "z": altitude})
        return route_points, True

    def _build_segment_route(self, start_xy, end_xy, start_z, end_z, margin):
        pending_points = [{"x": end_xy[0], "y": end_xy[1], "z": end_z}]
        routed_points = []
        current_xy = start_xy
        current_z = start_z
        max_iterations = max(8, len(self._obstacles) * 6)
        iteration = 0
        rerouted = False
        while pending_points and iteration < max_iterations:
            iteration += 1
            next_point = pending_points[0]
            next_xy = (next_point["x"], next_point["y"])
            blocking_obstacle = self._find_first_blocking_obstacle(
                current_xy,
                next_xy,
                margin,
                current_z,
                next_point["z"],
            )
            if blocking_obstacle is None:
                routed_points.append(dict(next_point))
                pending_points.pop(0)
                current_xy = next_xy
                current_z = next_point["z"]
                continue
            detour_points = self._build_detour_points(
                current_xy,
                next_xy,
                blocking_obstacle,
                margin,
                current_z,
                next_point["z"],
            )
            pending_points = [
                {"x": detour_x, "y": detour_y, "z": next_point["z"]}
                for detour_x, detour_y in detour_points
            ] + pending_points
            rerouted = True
        if pending_points:
            routed_points.extend(pending_points)
        return routed_points, rerouted

    def _dedupe_route_points(self, route_points, min_spacing=3.0):
        if not route_points:
            return []
        deduped = [dict(route_points[0])]
        for point in route_points[1:]:
            prev_point = deduped[-1]
            if math.hypot(point["x"] - prev_point["x"], point["y"] - prev_point["y"]) < min_spacing:
                deduped[-1] = dict(point)
                continue
            deduped.append(dict(point))
        return deduped

    def _compress_route_points(self, route_points, max_points=18):
        if len(route_points) <= max_points:
            return [dict(point) for point in route_points]
        kept = []
        last_index = len(route_points) - 1
        for target_index in range(max_points):
            sample_index = int(round((last_index * target_index) / float(max_points - 1)))
            point = route_points[sample_index]
            if kept and math.hypot(point["x"] - kept[-1]["x"], point["y"] - kept[-1]["y"]) < 1.0:
                continue
            kept.append(dict(point))
        if kept[-1] != route_points[-1]:
            kept[-1] = dict(route_points[-1])
        return kept

    def _compress_mission_points(self, route_points, max_points=16, preserve_prefix=4):
        if len(route_points) <= max_points:
            return [dict(point) for point in route_points]
        prefix_count = min(max(preserve_prefix, 0), len(route_points), max_points - 1)
        prefix = [dict(point) for point in route_points[:prefix_count]]
        remaining_slots = max_points - prefix_count
        tail = self._compress_route_points(route_points[prefix_count:], max_points=remaining_slots)
        kept = prefix
        for point in tail:
            if kept and math.hypot(point["x"] - kept[-1]["x"], point["y"] - kept[-1]["y"]) < 1.0:
                kept[-1] = dict(point)
            else:
                kept.append(dict(point))
        return kept[:max_points]

    def _refine_route_clearance(self, route_points, margin):
        if len(route_points) < 2 or not self._obstacles:
            return route_points
        refined = [dict(route_points[0])]
        rerouted = False
        for point in route_points[1:]:
            current = refined[-1]
            target_xy = self._push_point_out_of_obstacles((point["x"], point["y"]), margin, point["z"])
            segment_points, segment_rerouted = self._build_segment_route(
                (current["x"], current["y"]),
                target_xy,
                current["z"],
                point["z"],
                margin,
            )
            if segment_rerouted:
                rerouted = True
            refined.extend(segment_points)
        if rerouted:
            refined = self._dedupe_route_points(refined, min_spacing=4.0)
        return refined

    def _route_has_collisions(self, route_points, margin):
        if len(route_points) < 2:
            return False
        for first, second in zip(route_points[:-1], route_points[1:]):
            start_xy = (first["x"], first["y"])
            end_xy = (second["x"], second["y"])
            for obstacle in self._obstacles.values():
                if not obstacle.enabled:
                    continue
                if self._segment_hits_obstacle(start_xy, end_xy, obstacle, margin, first["z"], second["z"]):
                    return True
        return False

    def _make_route_conservative(self, route_points, base_margin, turn_radius, max_segment_length):
        safe_route = [dict(point) for point in route_points]
        for factor in (1.0, 1.2, 1.45):
            margin = base_margin * factor
            safe_route = self._refine_route_clearance(safe_route, margin)
            safe_route = self._smooth_route_points(safe_route, turn_radius * factor)
            safe_route = self._densify_route_points(safe_route, max_segment_length)
            if not self._route_has_collisions(safe_route, margin * 0.92):
                break
        return self._dedupe_route_points(safe_route, min_spacing=5.0)

    def _preview_start_point(self):
        if self._vehicle_positions:
            avg_x = sum(position[0] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
            avg_y = sum(position[1] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
            return {"x": avg_x, "y": avg_y, "z": float(self._anchor_z.value())}
        return {
            "x": float(self._anchor_x.value()),
            "y": float(self._anchor_y.value()),
            "z": float(self._anchor_z.value()),
        }

    def _prepend_preview_start(self, points):
        if not points:
            return points
        start_point = self._preview_start_point()
        first_point = points[0]
        if math.hypot(first_point["x"] - start_point["x"], first_point["y"] - start_point["y"]) < 5.0:
            return points
        return [start_point] + list(points)

    def _prepend_transition_regroup_point(self, route_points, formation_type, spacing):
        if not route_points or not self._vehicle_positions:
            return route_points
        avg_x = sum(position[0] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
        avg_y = sum(position[1] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
        if len(route_points) >= 2:
            next_point = route_points[1]
            heading_rad = math.atan2(next_point["y"] - route_points[0]["y"], next_point["x"] - route_points[0]["x"])
        else:
            heading_rad = math.radians(float(self._heading.value()))
        if formation_type == "column":
            transition_distance = max(spacing * 2.2, 90.0)
        elif formation_type == "v_shape":
            transition_distance = max(spacing * 3.4, 120.0)
        else:
            transition_distance = max(spacing * 3.0, 130.0)
        regroup_point = {
            "x": avg_x + math.cos(heading_rad) * transition_distance,
            "y": avg_y + math.sin(heading_rad) * transition_distance,
            "z": route_points[0]["z"],
        }
        first_point = route_points[0]
        if math.hypot(first_point["x"] - regroup_point["x"], first_point["y"] - regroup_point["y"]) < 12.0:
            return route_points
        return [regroup_point] + list(route_points)

    def _prepare_fixed_wing_mission_points(self, mission_points, formation_type, spacing, heading_deg):
        prepared = [dict(point) for point in mission_points]
        if not prepared or formation_type == "column":
            return prepared
        heading_rad = math.radians(heading_deg)
        forward = (math.cos(heading_rad), math.sin(heading_rad))
        min_route_length = max(spacing * 16.0, 560.0)
        if len(prepared) == 1:
            first_point = prepared[0]
            prepared.append(
                {
                    "x": first_point["x"] + forward[0] * min_route_length,
                    "y": first_point["y"] + forward[1] * min_route_length,
                    "z": first_point["z"],
                }
            )
            prepared.append(
                {
                    "x": first_point["x"] + forward[0] * (min_route_length + max(spacing * 10.0, 340.0)),
                    "y": first_point["y"] + forward[1] * (min_route_length + max(spacing * 10.0, 340.0)),
                    "z": first_point["z"],
                }
            )
            return prepared

        total_length = self._route_length_xy(prepared)
        if total_length < min_route_length:
            last_point = prepared[-1]
            prev_point = prepared[-2]
            dx = last_point["x"] - prev_point["x"]
            dy = last_point["y"] - prev_point["y"]
            segment_length = math.hypot(dx, dy)
            if segment_length > 1e-6:
                forward = (dx / segment_length, dy / segment_length)
            extension = min_route_length - total_length + max(spacing * 4.0, 140.0)
            prepared.append(
                {
                    "x": last_point["x"] + forward[0] * extension,
                    "y": last_point["y"] + forward[1] * extension,
                    "z": last_point["z"],
                }
            )
        return prepared

    def _prepend_launch_staging_points(self, route_points, formation_type, spacing):
        if not route_points or not self._vehicle_positions:
            self._launch_staging_points = []
            self._launch_staging_path = []
            return route_points
        avg_x = sum(position[0] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
        avg_y = sum(position[1] for position in self._vehicle_positions.values()) / float(len(self._vehicle_positions))
        start_point = {"x": avg_x, "y": avg_y, "z": route_points[0]["z"]}
        first_point = route_points[0]
        approach_dx = first_point["x"] - avg_x
        approach_dy = first_point["y"] - avg_y
        approach_distance = math.hypot(approach_dx, approach_dy)
        if approach_distance < max(spacing * 4.0, 160.0):
            if len(route_points) < 2:
                self._launch_staging_points = []
                self._launch_staging_path = []
                self._formation_stage_hold_distance = 0.0
                self._formation_stage_release_distance = 0.0
                return route_points
            near_first_point = True
            first_point = route_points[1]
            approach_dx = first_point["x"] - avg_x
            approach_dy = first_point["y"] - avg_y
            approach_distance = math.hypot(approach_dx, approach_dy)
        else:
            near_first_point = False
        if formation_type == "column":
            stage_fractions = (0.24, 0.52, 0.78)
        elif formation_type == "line":
            stage_fractions = (0.22, 0.48, 0.72)
        elif formation_type == "v_shape":
            stage_fractions = (0.24, 0.52, 0.78)
        else:
            stage_fractions = (0.22, 0.50, 0.75)
        route_tail = route_points[1:] if near_first_point else route_points
        if not route_tail:
            self._launch_staging_points = []
            self._launch_staging_path = []
            return route_points
        approach_route = self._build_safe_approach_route(start_point, route_tail[0], formation_type, spacing)
        approach_length = self._route_length_xy(approach_route)
        if approach_length < max(spacing * 3.0, 120.0):
            self._launch_staging_points = []
            self._launch_staging_path = []
            self._formation_stage_hold_distance = 0.0
            self._formation_stage_release_distance = 0.0
            return route_points
        stage_distances = [approach_length * fraction for fraction in stage_fractions]
        stage_points = [
            self._sample_point_at_route_distance_xy(approach_route, distance)
            for distance in stage_distances
        ]
        stage_points = self._dedupe_route_points(stage_points, min_spacing=8.0)
        if len(stage_points) < 3:
            self._launch_staging_points = []
            self._launch_staging_path = []
            self._formation_stage_hold_distance = 0.0
            self._formation_stage_release_distance = 0.0
            return route_points
        first_stage_distance = stage_distances[0]
        if formation_type == "column":
            self._formation_stage_hold_distance = 0.0
            self._formation_stage_release_distance = 0.0
        else:
            self._formation_stage_hold_distance = max(spacing * 0.35, 12.0)
            self._formation_stage_release_distance = max(
                stage_distances[2] - first_stage_distance,
                self._formation_stage_hold_distance + max(spacing * 2.2, 80.0),
            )
        stage_path = self._route_points_between_distances_xy(
            approach_route,
            stage_distances[0],
            stage_distances[2],
            extra_distances=stage_distances[1:2],
        )
        self._launch_staging_points = [dict(point) for point in stage_points]
        self._launch_staging_path = [dict(point) for point in stage_path]
        combined = [dict(point) for point in stage_path]
        approach_suffix = self._route_suffix_from_distance_xy(approach_route, stage_distances[2])
        for point in approach_suffix[1:] if approach_suffix else []:
            if math.hypot(point["x"] - combined[-1]["x"], point["y"] - combined[-1]["y"]) < 12.0:
                combined[-1] = dict(point)
            else:
                combined.append(dict(point))
        for point in route_tail[1:]:
            if math.hypot(point["x"] - combined[-1]["x"], point["y"] - combined[-1]["y"]) < 12.0:
                combined[-1] = dict(point)
            else:
                combined.append(dict(point))
        return combined

    def _build_safe_approach_route(self, start_point, target_point, formation_type, spacing):
        if self._planner_mode == "direct_no_avoid" or not self._obstacles:
            return [dict(start_point), dict(target_point)]
        margin = self._center_route_margin_for_scenario(formation_type, spacing)
        cell_size = max(min(spacing * 0.45, 26.0), 12.0)
        start_xy = self._push_point_out_of_obstacles(
            (start_point["x"], start_point["y"]),
            margin,
            start_point["z"],
        )
        target_xy = self._push_point_out_of_obstacles(
            (target_point["x"], target_point["y"]),
            margin,
            target_point["z"],
        )
        approach_points = [{"x": start_xy[0], "y": start_xy[1], "z": start_point["z"]}]
        segment_points, _segment_rerouted = self._astar_segment_route(
            start_xy,
            target_xy,
            target_point["z"],
            margin,
            cell_size,
            force_search=self._planner_mode == "global_astar",
        )
        approach_points.extend(segment_points)
        safe_route = self._make_route_conservative(
            approach_points,
            margin,
            max(spacing * 2.2, 72.0),
            max(spacing * 0.7, 16.0),
        )
        if not safe_route:
            return [dict(start_point), dict(target_point)]
        if math.hypot(safe_route[-1]["x"] - target_xy[0], safe_route[-1]["y"] - target_xy[1]) > 10.0:
            safe_route.append({"x": target_xy[0], "y": target_xy[1], "z": target_point["z"]})
        return self._dedupe_route_points(safe_route, min_spacing=5.0)

    def _densify_route_points(self, route_points, max_segment_length):
        if len(route_points) < 2:
            return route_points
        dense_points = [dict(route_points[0])]
        for first, second in zip(route_points[:-1], route_points[1:]):
            dx = second["x"] - first["x"]
            dy = second["y"] - first["y"]
            dz = second["z"] - first["z"]
            distance = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(distance / max(max_segment_length, 1.0))))
            for step in range(1, steps + 1):
                alpha = step / float(steps)
                dense_points.append(
                    {
                        "x": first["x"] + dx * alpha,
                        "y": first["y"] + dy * alpha,
                        "z": first["z"] + dz * alpha,
                    }
                )
        return self._dedupe_route_points(dense_points, min_spacing=max_segment_length * 0.25)

    def _smooth_route_points(self, route_points, turn_radius):
        route_points = self._dedupe_route_points(route_points)
        if len(route_points) < 3:
            return route_points
        smoothed = [dict(route_points[0])]
        for index in range(1, len(route_points) - 1):
            prev_point = route_points[index - 1]
            curr_point = route_points[index]
            next_point = route_points[index + 1]
            in_vec = (curr_point["x"] - prev_point["x"], curr_point["y"] - prev_point["y"])
            out_vec = (next_point["x"] - curr_point["x"], next_point["y"] - curr_point["y"])
            in_len = math.hypot(in_vec[0], in_vec[1])
            out_len = math.hypot(out_vec[0], out_vec[1])
            if in_len < 1e-6 or out_len < 1e-6:
                smoothed.append(dict(curr_point))
                continue
            in_dir = (in_vec[0] / in_len, in_vec[1] / in_len)
            out_dir = (out_vec[0] / out_len, out_vec[1] / out_len)
            dot_value = max(-1.0, min(1.0, in_dir[0] * out_dir[0] + in_dir[1] * out_dir[1]))
            corner_angle = math.acos(dot_value)
            if corner_angle < math.radians(12.0):
                smoothed.append(dict(curr_point))
                continue
            trim = min(turn_radius, in_len * 0.42, out_len * 0.42)
            if trim < 8.0:
                smoothed.append(dict(curr_point))
                continue
            entry = {
                "x": curr_point["x"] - in_dir[0] * trim,
                "y": curr_point["y"] - in_dir[1] * trim,
                "z": curr_point["z"],
            }
            exit_point = {
                "x": curr_point["x"] + out_dir[0] * trim,
                "y": curr_point["y"] + out_dir[1] * trim,
                "z": curr_point["z"],
            }
            smoothed.append(entry)
            sample_count = 5 if corner_angle > math.radians(45.0) else 3
            for sample_index in range(1, sample_count + 1):
                alpha = sample_index / float(sample_count + 1)
                raw_x = (1 - alpha) ** 2 * entry["x"] + 2 * (1 - alpha) * alpha * curr_point["x"] + alpha ** 2 * exit_point["x"]
                raw_y = (1 - alpha) ** 2 * entry["y"] + 2 * (1 - alpha) * alpha * curr_point["y"] + alpha ** 2 * exit_point["y"]
                safe_x, safe_y = self._push_point_out_of_obstacles(
                    (raw_x, raw_y),
                    max(turn_radius * 0.18, 10.0),
                    curr_point["z"],
                )
                smoothed.append({"x": safe_x, "y": safe_y, "z": curr_point["z"]})
            smoothed.append(exit_point)
        smoothed.append(dict(route_points[-1]))
        return self._dedupe_route_points(smoothed, min_spacing=4.0)

    def _vehicle_queue_rank(self, formation_type, spacing, vehicle_id):
        active_vehicle_ids = formation_vehicle_ids(formation_type, VEHICLE_IDS)
        offsets = list(zip(active_vehicle_ids, compute_formation_offsets(formation_type, spacing, active_vehicle_ids)))
        ordered = sorted(offsets, key=lambda item: (-item[1][0], abs(item[1][1]), item[0]))
        order_map = {name: index for index, (name, _offset) in enumerate(ordered)}
        return order_map.get(vehicle_id, active_vehicle_ids.index(vehicle_id) if vehicle_id in active_vehicle_ids else len(active_vehicle_ids))

    def _obstacle_interval_in_local_frame(self, obstacle, point_xy, heading_rad, margin):
        left_axis = (-math.sin(heading_rad), math.cos(heading_rad))
        forward_axis = (math.cos(heading_rad), math.sin(heading_rad))
        rel_center_x = obstacle.pose.position.x - point_xy[0]
        rel_center_y = obstacle.pose.position.y - point_xy[1]
        center_left = rel_center_x * left_axis[0] + rel_center_y * left_axis[1]
        center_forward = rel_center_x * forward_axis[0] + rel_center_y * forward_axis[1]
        half_left = abs(left_axis[0]) * obstacle.size.x / 2.0 + abs(left_axis[1]) * obstacle.size.y / 2.0 + margin
        half_forward = abs(forward_axis[0]) * obstacle.size.x / 2.0 + abs(forward_axis[1]) * obstacle.size.y / 2.0 + margin
        return {
            "l_min": center_left - half_left,
            "l_max": center_left + half_left,
            "f_min": center_forward - half_forward,
            "f_max": center_forward + half_forward,
        }

    def _corridor_profile_for_point(self, route_points, index, formation_type, spacing):
        offsets = compute_formation_offsets(formation_type, spacing, formation_vehicle_ids(formation_type, VEHICLE_IDS))
        base_lateral_span = max(abs(left_offset) for _forward_offset, left_offset in offsets) if offsets else 0.0
        if base_lateral_span < 1e-6:
            return {"scale": 1.0, "shift": 0.0, "single_file": False}
        if len(route_points) == 1:
            heading_rad = math.radians(float(self._heading.value()))
        elif index == len(route_points) - 1:
            prev_point = route_points[index - 1]
            curr_point = route_points[index]
            heading_rad = math.atan2(curr_point["y"] - prev_point["y"], curr_point["x"] - prev_point["x"])
        else:
            curr_point = route_points[index]
            next_point = route_points[index + 1]
            heading_rad = math.atan2(next_point["y"] - curr_point["y"], next_point["x"] - curr_point["x"])
        point = route_points[index]
        margin = max(spacing * 0.18, 8.0)
        forward_window = max(spacing * 1.2, 35.0)
        left_limit = None
        right_limit = None
        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            if not self._obstacle_affects_altitude(obstacle, point["z"], point["z"]):
                continue
            interval = self._obstacle_interval_in_local_frame(obstacle, (point["x"], point["y"]), heading_rad, margin)
            if interval["f_max"] < -forward_window or interval["f_min"] > forward_window:
                continue
            if interval["l_min"] <= 0.0 <= interval["l_max"]:
                return {"scale": 1.0, "shift": 0.0, "single_file": False}
            if interval["l_min"] >= 0.0:
                left_limit = interval["l_min"] if left_limit is None else min(left_limit, interval["l_min"])
            if interval["l_max"] <= 0.0:
                right_limit = interval["l_max"] if right_limit is None else max(right_limit, interval["l_max"])
        if left_limit is None or right_limit is None:
            return {"scale": 1.0, "shift": 0.0, "single_file": False}
        usable_left = max(0.0, left_limit - margin)
        usable_right = max(0.0, -right_limit - margin)
        corridor_half_width = min(usable_left, usable_right)
        if corridor_half_width >= base_lateral_span + spacing * 0.15:
            return {"scale": 1.0, "shift": 0.0, "single_file": False}
        scale = max(0.12, corridor_half_width / max(base_lateral_span, 1.0))
        shift = (usable_left - usable_right) * 0.35
        return {
            "scale": scale,
            "shift": shift,
            "single_file": scale < 0.42,
        }

    def _route_requires_corridor_compression(self, route_points, formation_type, spacing):
        for index in range(len(route_points)):
            profile = self._corridor_profile_for_point(route_points, index, formation_type, spacing)
            if profile["scale"] < 0.98:
                return True
        return False

    def _blended_corridor_profile(self, route_points, index, formation_type, spacing):
        weighted_scale = 0.0
        weighted_shift = 0.0
        total_weight = 0.0
        single_file = False
        for delta in range(-2, 3):
            sample_index = index + delta
            if sample_index < 0 or sample_index >= len(route_points):
                continue
            weight = 1.0 / (1.0 + abs(delta))
            profile = self._corridor_profile_for_point(route_points, sample_index, formation_type, spacing)
            weighted_scale += profile["scale"] * weight
            weighted_shift += profile["shift"] * weight
            total_weight += weight
            if profile["single_file"]:
                single_file = True
        if total_weight <= 1e-6:
            return {"scale": 1.0, "shift": 0.0, "single_file": False}
        return {
            "scale": weighted_scale / total_weight,
            "shift": weighted_shift / total_weight,
            "single_file": single_file,
        }

    def _compute_regroup_points(self, route_points, formation_type, spacing):
        regroup_points = []
        if len(route_points) < 3:
            return regroup_points
        previous_compressed = False
        for index in range(len(route_points)):
            profile = self._corridor_profile_for_point(route_points, index, formation_type, spacing)
            compressed = profile["scale"] < 0.98
            if previous_compressed and not compressed:
                regroup_points.append(dict(route_points[index]))
            previous_compressed = compressed
        return regroup_points

    def _route_requires_single_file(self, route_points, formation_type, spacing):
        for index in range(len(route_points)):
            profile = self._corridor_profile_for_point(route_points, index, formation_type, spacing)
            if profile["single_file"]:
                return True
        return False

    def _plan_center_route_preserving_formation(self, mission_points, formation_type, spacing):
        if self._planner_mode == "direct_no_avoid":
            self._regroup_points = []
            self._route_mode = "direct_no_avoid"
            return mission_points
        if not self._obstacles:
            self._regroup_points = []
            self._route_mode = "global_astar" if self._planner_mode == "global_astar" else "direct"
            return mission_points
        corridor_margin = self._center_route_margin_for_scenario(formation_type, spacing)
        cell_size = max(min(spacing * 0.45, 26.0), 12.0)
        first_x, first_y = self._push_point_out_of_obstacles(
            (mission_points[0]["x"], mission_points[0]["y"]),
            corridor_margin,
            mission_points[0]["z"],
        )
        planned_points = [dict(mission_points[0], x=first_x, y=first_y)]
        rerouted = False
        force_search = self._planner_mode == "global_astar"
        for task_point in mission_points[1:]:
            current_xy = (planned_points[-1]["x"], planned_points[-1]["y"])
            current_z = planned_points[-1]["z"]
            target_xy = self._push_point_out_of_obstacles((task_point["x"], task_point["y"]), corridor_margin, task_point["z"])
            target_z = task_point["z"]
            segment_points, segment_rerouted = self._astar_segment_route(
                current_xy,
                target_xy,
                target_z,
                corridor_margin,
                cell_size,
                force_search=force_search,
            )
            planned_points.extend(segment_points)
            rerouted = rerouted or segment_rerouted
        smoothed_points = self._make_route_conservative(
            planned_points,
            corridor_margin,
            max(spacing * 2.2, 72.0),
            max(spacing * 0.7, 16.0),
        )
        self._regroup_points = self._compute_regroup_points(smoothed_points, formation_type, spacing)
        if self._route_requires_single_file(smoothed_points, formation_type, spacing):
            self._route_mode = "single_file_regrouping"
        elif self._route_requires_corridor_compression(smoothed_points, formation_type, spacing):
            self._route_mode = "corridor_compressing"
        else:
            if self._planner_mode == "global_astar":
                self._route_mode = "global_astar"
            else:
                self._route_mode = "formation_preserving" if rerouted else "direct"
        return smoothed_points

    def _offset_route_for_vehicle(self, route_points, formation_type, spacing, vehicle_id):
        active_vehicle_ids = formation_vehicle_ids(formation_type, VEHICLE_IDS)
        offsets = dict(zip(active_vehicle_ids, compute_formation_offsets(formation_type, spacing, active_vehicle_ids)))
        forward_offset, left_offset = offsets.get(vehicle_id, (0.0, 0.0))
        if not route_points:
            return []
        queue_rank = self._vehicle_queue_rank(formation_type, spacing, vehicle_id)
        vehicle_points = []
        cumulative_distance = 0.0
        transition_distance = max(spacing * 4.0, 140.0)
        stage_hold_distance = float(getattr(self, "_formation_stage_hold_distance", 0.0))
        stage_release_distance = float(getattr(self, "_formation_stage_release_distance", 0.0))
        vehicle_error = float(getattr(self, "_live_vehicle_formation_errors", {}).get(vehicle_id, 0.0))
        layered_v_shape = formation_type == "v_shape" and self._scenario_key == "layered_altitude_demo"
        launch_staging_active = bool(getattr(self, "_launch_staging_points", []))
        if (
            not launch_staging_active
            and formation_type != "column"
            and stage_release_distance > stage_hold_distance > 0.0
            and vehicle_error > self._formation_error_threshold()
        ):
            hold_gain = 3.4 if layered_v_shape else 2.4
            release_gain = 5.8 if layered_v_shape else 4.2
            extra_hold = min(max(vehicle_error * hold_gain, spacing * (1.8 if layered_v_shape else 1.5)), max(spacing * (6.0 if layered_v_shape else 6.0), 220.0))
            extra_release = min(max(vehicle_error * release_gain, spacing * (3.2 if layered_v_shape else 2.5)), max(spacing * (10.0 if layered_v_shape else 10.0), 360.0))
            stage_hold_distance += extra_hold
            stage_release_distance += extra_release
        for index, point in enumerate(route_points):
            if index > 0:
                prev_point_for_distance = route_points[index - 1]
                cumulative_distance += math.hypot(
                    point["x"] - prev_point_for_distance["x"],
                    point["y"] - prev_point_for_distance["y"],
                )
            if len(route_points) == 1:
                heading_rad = math.radians(float(self._heading.value()))
            elif index == len(route_points) - 1:
                prev_point = route_points[index - 1]
                heading_rad = math.atan2(point["y"] - prev_point["y"], point["x"] - prev_point["x"])
            else:
                next_point = route_points[index + 1]
                heading_rad = math.atan2(next_point["y"] - point["y"], next_point["x"] - point["x"])
            corridor = self._blended_corridor_profile(route_points, index, formation_type, spacing)
            if formation_type == "column":
                transition_scale = 1.0
            else:
                if stage_release_distance > stage_hold_distance > 0.0:
                    if cumulative_distance <= stage_hold_distance:
                        transition_scale = 0.0
                    elif cumulative_distance >= stage_release_distance:
                        transition_scale = 1.0
                    else:
                        ramp_distance = max(stage_release_distance - stage_hold_distance, 1.0)
                        transition_scale = min(
                            1.0,
                            max(
                                0.0,
                                (cumulative_distance - stage_hold_distance) / float(max(ramp_distance, 1.0)),
                            ),
                        )
                else:
                    transition_scale = min(1.0, max(0.12, cumulative_distance / float(max(transition_distance, 1.0))))
            scaled_left = left_offset * corridor["scale"] * transition_scale + corridor["shift"]
            scaled_forward = forward_offset * transition_scale
            if formation_type == "v_shape":
                scaled_left *= 1.15 if layered_v_shape else 1.10
                scaled_forward *= 0.90 if layered_v_shape else 0.92
            if corridor["single_file"]:
                scaled_left *= max(0.03, corridor["scale"] * 0.45)
                scaled_forward -= queue_rank * max(spacing, 22.0) * (1.0 - corridor["scale"])
            forward = (math.cos(heading_rad), math.sin(heading_rad))
            left = (-forward[1], forward[0])
            vehicle_points.append(
                {
                    "x": point["x"] + forward[0] * scaled_forward + left[0] * scaled_left,
                    "y": point["y"] + forward[1] * scaled_forward + left[1] * scaled_left,
                    "z": point["z"],
                }
            )
        return vehicle_points

    def _plan_task_points_with_avoidance(self, mission_points):
        if self._planner_mode == "direct_no_avoid":
            return mission_points
        if not self._obstacles:
            return mission_points
        cell_size = max(min(float(self._spacing.value()) * 0.4, 22.0), 10.0)
        first_x, first_y = self._push_point_out_of_obstacles(
            (mission_points[0]["x"], mission_points[0]["y"]),
            max(float(self._spacing.value()) * 0.9, 40.0),
            mission_points[0]["z"],
        )
        planned_points = [dict(mission_points[0], x=first_x, y=first_y)]
        safety_margin = max(float(self._spacing.value()) * 0.9, 40.0)
        force_search = self._planner_mode == "global_astar"
        for task_point in mission_points[1:]:
            current_xy = (planned_points[-1]["x"], planned_points[-1]["y"])
            current_z = planned_points[-1]["z"]
            target_z = task_point["z"]
            target_xy = self._push_point_out_of_obstacles((task_point["x"], task_point["y"]), safety_margin, target_z)
            segment_points, inserted = self._astar_segment_route(
                current_xy,
                target_xy,
                target_z,
                safety_margin,
                cell_size,
                force_search=force_search,
            )
            planned_points.extend(segment_points)
        return self._make_route_conservative(
            planned_points,
            safety_margin,
            max(float(self._spacing.value()) * 1.9, 58.0),
            max(float(self._spacing.value()) * 0.55, 13.0),
        )

    def _update_route_preview(self):
        requested = [
            {"x": point["x"], "y": point["y"], "z": point["z"]}
            for point in (self._task_points or [{"x": float(self._anchor_x.value()), "y": float(self._anchor_y.value()), "z": float(self._anchor_z.value())}])
        ]
        preview_start = self._preview_start_point()
        preview_route_points = [preview_start] + requested if requested else [preview_start]
        safe_delta_x = self._safe_anchor_xy[0] - float(self._anchor_x.value())
        safe_delta_y = self._safe_anchor_xy[1] - float(self._anchor_y.value())
        adjusted = [
            {"x": point["x"] + safe_delta_x, "y": point["y"] + safe_delta_y, "z": point["z"]}
            for point in preview_route_points
        ]
        planned = self._plan_center_route_preserving_formation(
            adjusted,
            self._formation_type.currentData(),
            float(self._spacing.value()),
        )
        self._requested_task_path = preview_route_points
        self._planned_task_path = planned
        self._update_layered_demo_visuals()
        self._refresh_situation_view()
        self._update_metric_summary()

    def _route_length_xy(self, points):
        total = 0.0
        for first, second in zip(points[:-1], points[1:]):
            total += math.hypot(second["x"] - first["x"], second["y"] - first["y"])
        return total

    def _sample_point_at_route_distance_xy(self, points, target_distance):
        if not points:
            return None
        if len(points) == 1:
            return dict(points[0])
        target_distance = max(0.0, target_distance)
        walked = 0.0
        for first, second in zip(points[:-1], points[1:]):
            segment = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            if walked + segment >= target_distance and segment > 1e-6:
                alpha = (target_distance - walked) / segment
                return {
                    "x": first["x"] + alpha * (second["x"] - first["x"]),
                    "y": first["y"] + alpha * (second["y"] - first["y"]),
                    "z": first.get("z", 0.0) + alpha * (second.get("z", 0.0) - first.get("z", 0.0)),
                }
            walked += segment
        return dict(points[-1])

    def _route_suffix_from_distance_xy(self, points, start_distance):
        if not points:
            return []
        if len(points) == 1:
            return [dict(points[0])]
        suffix_start = self._sample_point_at_route_distance_xy(points, start_distance)
        suffix = [suffix_start] if suffix_start is not None else []
        walked = 0.0
        appended = False
        for first, second in zip(points[:-1], points[1:]):
            segment = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            segment_end = walked + segment
            if segment_end >= start_distance:
                appended = True
            if appended and (not suffix or math.hypot(second["x"] - suffix[-1]["x"], second["y"] - suffix[-1]["y"]) >= 8.0):
                suffix.append(dict(second))
            walked = segment_end
        return suffix

    def _route_points_between_distances_xy(self, points, start_distance, end_distance, extra_distances=None):
        if not points:
            return []
        start_distance = max(0.0, start_distance)
        end_distance = max(start_distance, end_distance)
        total_length = self._route_length_xy(points)
        end_distance = min(end_distance, total_length)
        desired_distances = [start_distance, end_distance]
        for distance in extra_distances or []:
            if start_distance < distance < end_distance:
                desired_distances.append(distance)
        walked = 0.0
        for first, second in zip(points[:-1], points[1:]):
            segment = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            walked += segment
            if start_distance < walked < end_distance:
                desired_distances.append(walked)
        sampled = []
        for distance in sorted(desired_distances):
            point = self._sample_point_at_route_distance_xy(points, distance)
            if point is None:
                continue
            if sampled and math.hypot(point["x"] - sampled[-1]["x"], point["y"] - sampled[-1]["y"]) < 5.0:
                sampled[-1] = dict(point)
            else:
                sampled.append(dict(point))
        return sampled

    def _sample_point_along_route_xy(self, points, progress_ratio):
        if not points:
            return None
        if len(points) == 1:
            return dict(points[0])
        progress_ratio = min(max(progress_ratio, 0.0), 1.0)
        total_length = self._route_length_xy(points)
        if total_length <= 1e-6:
            return dict(points[-1])
        target = total_length * progress_ratio
        walked = 0.0
        for first, second in zip(points[:-1], points[1:]):
            segment = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            if walked + segment >= target and segment > 1e-6:
                alpha = (target - walked) / segment
                return {
                    "x": first["x"] + alpha * (second["x"] - first["x"]),
                    "y": first["y"] + alpha * (second["y"] - first["y"]),
                    "z": first.get("z", 0.0) + alpha * (second.get("z", 0.0) - first.get("z", 0.0)),
                }
            walked += segment
        return dict(points[-1])

    def _update_layered_demo_visuals(self):
        if self._scenario_key != "layered_altitude_demo":
            return
        requested_display = self._display_requested_path()
        planned_display = self._display_planned_path()
        if len(planned_display) < 2:
            return

        self._scenario_comparison_routes = {
            "unlayered": [dict(point) for point in requested_display] if requested_display else [],
            "layered": [dict(point) for point in planned_display],
        }

        zone_center = self._sample_point_along_route_xy(planned_display, 0.42)
        if zone_center is None:
            return
        self._scenario_layering_zone = {
            "center_x": float(zone_center["x"]),
            "center_y": float(zone_center["y"]),
            "width": 160.0,
            "height": 120.0,
        }

    def _display_planned_path(self):
        planned = [dict(point) for point in (self._planned_task_path or [])]
        if not planned:
            return planned
        staging_points = [dict(point) for point in (self._launch_staging_points or [])]
        if not staging_points:
            return planned
        last_stage = staging_points[-1]
        best_index = None
        best_distance = float("inf")
        scan_limit = min(len(planned), max(len(getattr(self, "_launch_staging_path", [])) + 6, len(staging_points) + 12, 16))
        for index, point in enumerate(planned[:scan_limit]):
            distance = math.hypot(point["x"] - last_stage["x"], point["y"] - last_stage["y"])
            if distance < best_distance:
                best_distance = distance
                best_index = index
        if best_index is not None and best_distance < 12.0:
            return [dict(last_stage)] + [dict(point) for point in planned[best_index + 1 :]]
        return planned

    def _display_requested_path(self):
        requested = [dict(point) for point in (self._requested_task_path or [])]
        if not requested:
            return requested
        staging_points = [dict(point) for point in (self._launch_staging_points or [])]
        if not staging_points:
            return requested
        last_stage = staging_points[-1]
        remaining = requested[1:] if len(requested) > 1 else []
        if remaining:
            first_remaining = remaining[0]
            if math.hypot(first_remaining["x"] - last_stage["x"], first_remaining["y"] - last_stage["y"]) < 8.0:
                remaining[0] = last_stage
                return remaining
        return [last_stage] + remaining

    def _build_mission_items(self, route_with_start, fix, altitude, landing_forward=None, include_takeoff=True):
        if not route_with_start or len(route_with_start) < 2:
            return []
        local_pose = route_with_start[0]
        mission_points = self._compress_mission_points(route_with_start[1:], max_points=16, preserve_prefix=4)
        if not mission_points:
            return []
        if landing_forward is None:
            heading_rad = math.radians(float(self._heading.value()))
            landing_forward = (math.cos(heading_rad), math.sin(heading_rad))
        mission_items = []
        first_point = mission_points[0]
        takeoff_dx = first_point["x"] - local_pose["x"]
        takeoff_dy = first_point["y"] - local_pose["y"]
        takeoff_lat, takeoff_lon = self._offset_fix(fix.latitude, fix.longitude, takeoff_dx, takeoff_dy)
        if include_takeoff:
            mission_items.append(
                self._make_waypoint(
                    command=22,
                    latitude=takeoff_lat,
                    longitude=takeoff_lon,
                    altitude=max(first_point["z"], 40.0),
                    is_current=True,
                    param1=15.0,
                )
            )
        else:
            mission_items.append(
                self._make_waypoint(
                    command=16,
                    latitude=takeoff_lat,
                    longitude=takeoff_lon,
                    altitude=first_point["z"],
                    is_current=True,
                    param1=0.0,
                )
            )
        for task_point in mission_points:
            if task_point is first_point:
                continue
            dx = task_point["x"] - local_pose["x"]
            dy = task_point["y"] - local_pose["y"]
            target_lat, target_lon = self._offset_fix(fix.latitude, fix.longitude, dx, dy)
            mission_items.append(
                self._make_waypoint(
                    command=16,
                    latitude=target_lat,
                    longitude=target_lon,
                    altitude=task_point["z"],
                    is_current=False,
                    param1=0.0,
                )
            )
        final_point = mission_points[-1]
        landing_altitude = max(final_point["z"] * 0.2, 10.0)
        glide_delta = max(final_point["z"] - landing_altitude, 5.0)
        landing_distance = max(glide_delta / math.tan(math.radians(8.0)), 450.0)
        landing_target_x = final_point["x"] + landing_forward[0] * landing_distance
        landing_target_y = final_point["y"] + landing_forward[1] * landing_distance
        landing_dx = landing_target_x - local_pose["x"]
        landing_dy = landing_target_y - local_pose["y"]
        landing_lat, landing_lon = self._offset_fix(fix.latitude, fix.longitude, landing_dx, landing_dy)
        mission_items.append(
            self._make_waypoint(
                command=21,
                latitude=landing_lat,
                longitude=landing_lon,
                altitude=landing_altitude,
                is_current=False,
                param1=0.0,
            )
        )
        return mission_items

    def _make_waypoint(self, command, latitude, longitude, altitude, is_current, param1=0.0):
        waypoint = Waypoint()
        waypoint.frame = 3
        waypoint.command = command
        waypoint.is_current = is_current
        waypoint.autocontinue = True
        waypoint.param1 = param1
        waypoint.param2 = 15.0
        waypoint.param3 = 0.0
        waypoint.param4 = float("nan")
        waypoint.x_lat = latitude
        waypoint.y_long = longitude
        waypoint.z_alt = altitude
        return waypoint

    def _set_param_int(self, vehicle_id, param_name, param_value):
        service_name = "/{}/mavros/param/set".format(vehicle_id)
        if vehicle_id not in self._param_set_clients:
            rospy.wait_for_service(service_name, timeout=1.0)
            self._param_set_clients[vehicle_id] = rospy.ServiceProxy(service_name, ParamSet)
        value = ParamValue()
        value.integer = int(param_value)
        value.real = 0.0
        return self._param_set_clients[vehicle_id](param_name, value)

    def _apply_prearm_params(self, vehicle_id):
        if not self._enable_prearm_param_push:
            return
        for param_name, param_value in (
            ("SYS_HAS_NUM_ASPD", 0),
            ("CBRK_SUPPLY_CHK", 894281),
            ("NAV_DLL_ACT", 0),
        ):
            try:
                self._set_param_int(vehicle_id, param_name, param_value)
                rospy.sleep(0.05)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("Prearm param set failed %s %s=%s: %s", vehicle_id, param_name, param_value, exc)
                break

    def _offset_fix(self, latitude_deg, longitude_deg, east_m, north_m):
        lat_scale = 111320.0
        lon_scale = max(111320.0 * math.cos(math.radians(latitude_deg)), 1.0)
        return latitude_deg + north_m / lat_scale, longitude_deg + east_m / lon_scale

    def _call_arm_service_worker(self, arm_value):
        result = True
        for vehicle_id in VEHICLE_IDS:
            service_name = "/{}/mavros/cmd/arming".format(vehicle_id)
            try:
                if arm_value:
                    self._apply_prearm_params(vehicle_id)
                if vehicle_id not in self._arming_clients:
                    rospy.wait_for_service(service_name, timeout=1.0)
                    self._arming_clients[vehicle_id] = rospy.ServiceProxy(service_name, CommandBool)
                response = self._arming_clients[vehicle_id](arm_value)
                result = result and bool(response.success)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                result = False
                rospy.logwarn("解锁服务调用失败 %s: %s", vehicle_id, exc)
        self.action_result_signal.emit(
            "集群状态: {} {}".format("解锁" if arm_value else "上锁", "成功" if result else "部分失败")
        )

        if result:
            self._publish_mission_phase("armed" if arm_value else "idle")

    def _call_mode_service(self, custom_mode):
        threading.Thread(target=self._call_mode_service_worker, args=(custom_mode,), daemon=True).start()

    def _call_mode_service_worker(self, custom_mode):
        result = True
        for vehicle_id in VEHICLE_IDS:
            service_name = "/{}/mavros/set_mode".format(vehicle_id)
            try:
                if vehicle_id not in self._mode_clients:
                    rospy.wait_for_service(service_name, timeout=1.0)
                    self._mode_clients[vehicle_id] = rospy.ServiceProxy(service_name, SetMode)
                response = self._mode_clients[vehicle_id](0, custom_mode)
                result = result and bool(response.mode_sent)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                result = False
                rospy.logwarn("模式服务调用失败 %s: %s", vehicle_id, exc)
        mode_label = {"AUTO.MISSION": "任务模式", "AUTO.LOITER": "盘旋待命"}.get(
            custom_mode, custom_mode
        )
        self.action_result_signal.emit("集群状态: {} {}".format(mode_label, "已下发" if result else "部分失败"))

        if result and custom_mode == "AUTO.LOITER":
            self._publish_mission_phase("holding")

    def _on_status(self, msg):
        self.status_signal.emit(msg)

    def _handle_status_update(self, msg):
        if self._vehicle_table is None:
            return
        self._status_label.setText(
            "集群状态: {state} | 当前队形: {formation} | 飞机数: {count}".format(
                state=msg.state or "未知",
                formation=formation_label(msg.active_formation or "-"),
                count=len(msg.vehicle_ids) if msg.vehicle_ids else len(VEHICLE_IDS),
            )
        )
        if msg.active_formation:
            self._active_formation = msg.active_formation
        self._mission_phase = msg.state or self._mission_phase
        self._update_workflow_status()
        for vehicle_id in VEHICLE_IDS:
            self._update_vehicle_row(vehicle_id, default_state=msg.state or "unknown")
        self._refresh_situation_view()
        self._refresh_live_metrics()

    def _on_formation_cmd(self, msg):
        self.formation_signal.emit(msg)

    def _handle_formation_update(self, msg):
        if self._vehicle_table is None:
            return
        self._active_formation = msg.formation_type or self._active_formation
        self._active_spacing = msg.spacing or self._active_spacing
        self._active_heading = msg.heading_deg
        self._anchor_xy = (msg.anchor.pose.position.x, msg.anchor.pose.position.y)
        for index in range(self._formation_type.count()):
            if self._formation_type.itemData(index) == self._active_formation:
                self._formation_type.setCurrentIndex(index)
                break
        self._spacing.setValue(self._active_spacing)
        self._heading.setValue(self._active_heading)
        self._anchor_x.setValue(self._anchor_xy[0])
        self._anchor_y.setValue(self._anchor_xy[1])
        self._anchor_z.setValue(msg.anchor.pose.position.z or self._anchor_z.value())
        self._refresh_situation_view()
        self._refresh_live_metrics()

    def _on_safe_formation_cmd(self, msg):
        self.safe_formation_signal.emit(msg)

    def _handle_safe_formation_update(self, msg):
        if self._situation_view is None:
            return
        self._safe_anchor_xy = (msg.anchor.pose.position.x, msg.anchor.pose.position.y)
        self._update_route_preview()
        self._refresh_situation_view()
        self._refresh_live_metrics()

    def _on_obstacle(self, msg):
        self.obstacle_signal.emit(msg)

    def _handle_obstacle_update(self, msg):
        if self._situation_view is None:
            return
        self._obstacles[msg.id] = msg
        if msg.enabled:
            if msg.id not in self._obstacle_order:
                self._obstacle_order.append(msg.id)
        else:
            self._obstacle_order = [obstacle_id for obstacle_id in self._obstacle_order if obstacle_id != msg.id]
        self._update_route_preview()
        self._refresh_situation_view()
        self._refresh_live_metrics()

    def _on_avoidance_status(self, msg):
        self.avoidance_signal.emit(msg)

    def _handle_avoidance_update(self, msg):
        if self._situation_label is None:
            return
        self._avoidance_state = {"clear": "无冲突", "rerouted": "已绕障"}.get(msg.state, msg.state or "无冲突")
        self._situation_label.setText(
            "二维态势图 | 避障: {state} | 安全队形: {formation}".format(
                state=self._avoidance_state,
                formation=formation_label(msg.active_formation or self._active_formation),
            )
        )
        self._refresh_situation_view()

    def _on_gazebo_status(self, msg):
        self.gazebo_signal.emit(msg.data)

    def _handle_gazebo_status(self, status_text):
        status_map = {
            "formation:connected": "Gazebo联动: 编队标记已连接",
            "formation:unavailable": "Gazebo联动: 编队服务未接通",
            "formation:error": "Gazebo联动: 编队同步异常",
            "obstacle:connected": "Gazebo联动: 障碍模型已连接",
            "obstacle:unavailable": "Gazebo联动: 障碍服务未接通",
            "obstacle:error": "Gazebo联动: 障碍同步异常",
        }
        self._gazebo_label.setText(status_map.get(status_text, "Gazebo联动: {}".format(status_text)))

    def _handle_pose_update(self, vehicle_id, x_value, y_value, z_value):
        self._vehicle_positions[vehicle_id] = (x_value, y_value, z_value)
        self._vehicle_last_seen[vehicle_id] = time.time()
        self._update_vehicle_row(vehicle_id)
        self._refresh_situation_view()
        self._refresh_live_metrics()

    def _update_vehicle_row(self, vehicle_id, default_state="tracking"):
        if self._vehicle_table is None:
            return
        row = VEHICLE_IDS.index(vehicle_id)
        last_seen = self._vehicle_last_seen.get(vehicle_id)
        if last_seen is None:
            state_text = "waiting"
            x_text = "-"
            y_text = "-"
        else:
            state_text = default_state if time.time() - last_seen < 2.0 else "超时"
            x_text = "{:.1f}".format(self._vehicle_positions[vehicle_id][0])
            y_text = "{:.1f}".format(self._vehicle_positions[vehicle_id][1])
        state_map = {"waiting": "等待数据", "tracking": "跟踪中", "idle": "空闲", "clear": "无冲突", "rerouted": "已绕障", "unknown": "未知"}
        state_map = {
            "waiting": "等待数据",
            "tracking": "跟踪中",
            "idle": "空闲",
            "formation_ready": "编队已配置",
            "mission_uploaded": "任务已上传",
            "arming": "正在解锁",
            "armed": "已解锁",
            "mission_starting": "正在开始任务",
            "mission_active": "任务执行中",
            "holding": "盘旋待命",
            "clear": "无冲突",
            "rerouted": "已绕障",
            "unknown": "未知",
        }
        self._vehicle_table.item(row, 1).setText(state_text)
        self._vehicle_table.item(row, 1).setText(state_map.get(state_text, state_text))
        self._vehicle_table.item(row, 2).setText(formation_label(self._active_formation))
        self._vehicle_table.item(row, 3).setText(x_text)
        self._vehicle_table.item(row, 4).setText(y_text)

    def _refresh_situation_view(self):
        if self._situation_view is None:
            return
        self._situation_view.update_scene(
            positions={vehicle_id: (value[0], value[1]) for vehicle_id, value in self._vehicle_positions.items()},
            obstacles=self._obstacles,
            formation_type=self._active_formation,
            spacing=self._active_spacing,
            heading_deg=self._active_heading,
            anchor=self._anchor_xy,
            safe_anchor=self._safe_anchor_xy,
            avoidance_state=self._avoidance_state,
            requested_path=self._display_requested_path(),
            planned_path=self._display_planned_path(),
            regroup_points=self._regroup_points,
            staging_points=self._launch_staging_points,
            staging_path=self._launch_staging_path,
            layering_zone=self._scenario_layering_zone,
            altitude_assignments=self._scenario_altitude_assignments,
            altitude_levels=self._scenario_altitude_levels,
            comparison_routes=self._scenario_comparison_routes,
            vehicle_layer_tags=self._scenario_layer_tags(),
            field_region={
                "center_x": float(self._obs_field_cx.value()),
                "center_y": float(self._obs_field_cy.value()),
                "width": float(self._obs_field_w.value()),
                "height": float(self._obs_field_hh.value()),
            },
            route_mode=self._route_mode,
        )
        self._situation_view.vehicle_errors = dict(self._live_vehicle_formation_errors)
        self._situation_view.update()

    def shutdown_plugin(self):
        self._status_sub.unregister()
        self._formation_sub.unregister()
        self._safe_formation_sub.unregister()
        self._obstacle_sub.unregister()
        self._avoidance_status_sub.unregister()
        self._gazebo_status_sub.unregister()
        for subscriber in self._pose_subscribers:
            subscriber.unregister()


class SwarmControlPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SwarmControlPlugin")
        self._widget = SwarmControlWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
