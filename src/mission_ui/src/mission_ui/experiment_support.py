#!/usr/bin/env python3
import math
from pathlib import Path


PLANNER_MODE_OPTIONS = [
    ("当前策略", "adaptive"),
    ("直达不避障", "direct_no_avoid"),
    ("简化A*全局", "global_astar"),
]


SCENARIO_PRESETS = {
    "urban_delivery": {
        "label": "城市障碍场",
        "description": "典型城市障碍分布，验证绕行与局部重规划能力。",
        "formation_type": "line",
        "spacing": 32.0,
        "heading_deg": 75.0,
        "anchor": (420.0, -40.0, 80.0),
        "task_points": [
            {"x": 500.0, "y": 20.0, "z": 80.0},
            {"x": 660.0, "y": 120.0, "z": 80.0},
            {"x": 810.0, "y": 210.0, "z": 80.0},
        ],
        "obstacles": [
            {"id": "scenario_obstacle_01", "shape": "box", "x": 550.0, "y": 10.0, "z": 0.0, "sx": 60.0, "sy": 80.0, "h": 140.0},
            {"id": "scenario_obstacle_02", "shape": "box", "x": 610.0, "y": 100.0, "z": 0.0, "sx": 70.0, "sy": 60.0, "h": 120.0},
            {"id": "scenario_obstacle_03", "shape": "cylinder", "x": 705.0, "y": 155.0, "z": 0.0, "sx": 50.0, "sy": 50.0, "h": 130.0},
            {"id": "scenario_obstacle_04", "shape": "box", "x": 760.0, "y": 250.0, "z": 0.0, "sx": 65.0, "sy": 85.0, "h": 150.0},
        ],
    },
    "corridor_passage": {
        "label": "通道场",
        "description": "狭窄通道场景，验证中缝穿越、通道压缩和单列重组能力。",
        "formation_type": "line",
        "spacing": 35.0,
        "heading_deg": 90.0,
        "anchor": (380.0, 0.0, 75.0),
        "task_points": [
            {"x": 460.0, "y": 0.0, "z": 75.0},
            {"x": 760.0, "y": 0.0, "z": 75.0},
        ],
        "obstacles": [
            {"id": "scenario_obstacle_01", "shape": "box", "x": 505.0, "y": 72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
            {"id": "scenario_obstacle_02", "shape": "box", "x": 505.0, "y": -72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
            {"id": "scenario_obstacle_03", "shape": "box", "x": 575.0, "y": 72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
            {"id": "scenario_obstacle_04", "shape": "box", "x": 575.0, "y": -72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
            {"id": "scenario_obstacle_05", "shape": "box", "x": 645.0, "y": 72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
            {"id": "scenario_obstacle_06", "shape": "box", "x": 645.0, "y": -72.0, "z": 0.0, "sx": 62.0, "sy": 56.0, "h": 120.0},
        ],
    },
    "dynamic_reconfiguration": {
        "label": "动态改编队场",
        "description": "任务执行中切换为V形编队，验证重组与恢复能力。",
        "formation_type": "v_shape",
        "spacing": 24.0,
        "heading_deg": 135.0,
        "anchor": (520.0, 140.0, 60.0),
        "task_points": [
            {"x": 520.0, "y": 140.0, "z": 60.0},
            {"x": 590.0, "y": 220.0, "z": 60.0},
            {"x": 670.0, "y": 305.0, "z": 60.0},
        ],
        "obstacles": [
            {"id": "scenario_obstacle_01", "shape": "box", "x": 570.0, "y": 195.0, "z": 0.0, "sx": 55.0, "sy": 48.0, "h": 120.0},
            {"id": "scenario_obstacle_02", "shape": "box", "x": 635.0, "y": 270.0, "z": 0.0, "sx": 55.0, "sy": 48.0, "h": 120.0},
        ],
    },
    "layered_altitude_demo": {
        "label": "高度分层场景",
        "description": "在规划主路上的重组段引入临时高度层，用于展示不分层与分层后任务组织差异。",
        "formation_type": "v_shape",
        "spacing": 30.0,
        "heading_deg": 60.0,
        "anchor": (560.0, 40.0, 80.0),
        "task_points": [
            {"x": 660.0, "y": 135.0, "z": 80.0},
            {"x": 780.0, "y": 235.0, "z": 80.0},
            {"x": 940.0, "y": 355.0, "z": 80.0},
        ],
        "obstacles": [
            {"id": "scenario_obstacle_01", "shape": "box", "x": 670.0, "y": 145.0, "z": 0.0, "sx": 62.0, "sy": 78.0, "h": 135.0},
            {"id": "scenario_obstacle_02", "shape": "box", "x": 760.0, "y": 225.0, "z": 0.0, "sx": 68.0, "sy": 60.0, "h": 130.0},
            {"id": "scenario_obstacle_03", "shape": "cylinder", "x": 855.0, "y": 315.0, "z": 0.0, "sx": 50.0, "sy": 50.0, "h": 125.0},
        ],
        "layering_zone": {"center_x": 725.0, "center_y": 225.0, "width": 180.0, "height": 130.0},
        "altitude_levels": {"upper": 100.0, "base": 80.0, "lower": 60.0},
        "altitude_assignments": {
            "upper": ["uav_2", "uav_4"],
            "base": ["uav_1"],
            "lower": ["uav_3", "uav_5"],
        },
        "comparison_routes": {
            "unlayered": [
                {"x": 560.0, "y": 40.0, "z": 80.0},
                {"x": 665.0, "y": 120.0, "z": 80.0},
                {"x": 770.0, "y": 220.0, "z": 80.0},
                {"x": 940.0, "y": 355.0, "z": 80.0},
            ],
            "layered": [
                {"x": 560.0, "y": 40.0, "z": 80.0},
                {"x": 635.0, "y": 105.0, "z": 80.0},
                {"x": 710.0, "y": 185.0, "z": 100.0},
                {"x": 795.0, "y": 250.0, "z": 80.0},
                {"x": 940.0, "y": 355.0, "z": 80.0},
            ],
        },
    },
    "multi_point_patrol": {
        "label": "多点任务场",
        "description": "大范围多航点巡回任务，验证全局任务组织、折返衔接与综合指标表现。",
        "formation_type": "echelon_left",
        "spacing": 28.0,
        "heading_deg": 30.0,
        "anchor": (410.0, -120.0, 85.0),
        "task_points": [
            {"x": 500.0, "y": -60.0, "z": 85.0},
            {"x": 620.0, "y": 100.0, "z": 85.0},
            {"x": 830.0, "y": 155.0, "z": 85.0},
            {"x": 930.0, "y": 0.0, "z": 85.0},
            {"x": 770.0, "y": -190.0, "z": 85.0},
            {"x": 540.0, "y": -165.0, "z": 85.0},
        ],
        "obstacles": [
            {"id": "scenario_obstacle_01", "shape": "box", "x": 650.0, "y": 35.0, "z": 0.0, "sx": 55.0, "sy": 70.0, "h": 130.0},
            {"id": "scenario_obstacle_02", "shape": "cylinder", "x": 840.0, "y": -95.0, "z": 0.0, "sx": 42.0, "sy": 42.0, "h": 120.0},
        ],
    },
}


def planner_mode_label(planner_mode):
    for label, value in PLANNER_MODE_OPTIONS:
        if value == planner_mode:
            return label
    return planner_mode or "当前策略"


def scenario_options():
    return [(value["label"], key) for key, value in SCENARIO_PRESETS.items()]


def get_scenario_preset(key):
    return SCENARIO_PRESETS.get(key)


def workspace_root_from_file(file_path):
    return Path(file_path).resolve().parents[4]


def polyline_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for first, second in zip(points[:-1], points[1:]):
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        dz = second["z"] - first["z"]
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def estimate_route_energy_wh(points, mass_factor=1.0):
    if not points:
        return 0.0
    takeoff_wh = 14.0 * mass_factor
    cruise_wh_per_m = 0.055 * mass_factor
    climb_wh_per_m = 0.11 * mass_factor
    descent_wh_per_m = 0.035 * mass_factor
    turn_penalty_scale = 3.8 * mass_factor

    total = takeoff_wh
    previous_heading = None
    for first, second in zip(points[:-1], points[1:]):
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        dz = second["z"] - first["z"]
        horizontal = math.hypot(dx, dy)
        total += horizontal * cruise_wh_per_m
        if dz >= 0.0:
            total += dz * climb_wh_per_m
        else:
            total += abs(dz) * descent_wh_per_m
        heading = math.atan2(dy, dx) if horizontal > 1e-6 else previous_heading
        if previous_heading is not None and heading is not None:
            delta = abs(math.atan2(math.sin(heading - previous_heading), math.cos(heading - previous_heading)))
            total += delta * turn_penalty_scale
        previous_heading = heading
    return total


def point_clearance_to_obstacle(point, obstacle, extra_margin=0.0):
    half_x = obstacle["sx"] / 2.0 + extra_margin
    half_y = obstacle["sy"] / 2.0 + extra_margin
    dx = abs(point["x"] - obstacle["x"]) - half_x
    dy = abs(point["y"] - obstacle["y"]) - half_y
    if dx <= 0.0 and dy <= 0.0:
        return -min(abs(dx), abs(dy))
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def min_clearance_to_obstacles(points, obstacles, extra_margin=0.0):
    if not points or not obstacles:
        return float("inf")
    min_clearance = float("inf")
    for point in points:
        for obstacle in obstacles:
            min_clearance = min(min_clearance, point_clearance_to_obstacle(point, obstacle, extra_margin=extra_margin))
    return min_clearance
