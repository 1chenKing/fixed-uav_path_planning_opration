import math
from pathlib import Path


def run():
    import sys

    repo_src = "/home/chen/catkin_ws/src/mission_ui/src"
    if repo_src not in sys.path:
        sys.path.insert(0, repo_src)

    import mission_ui.swarm_control_plugin as plugin
    from mission_ui.swarm_control_plugin import SwarmControlWidget, VEHICLE_IDS, compute_formation_offsets

    class DummyValue:
        def __init__(self, value):
            self._value = value

        def value(self):
            return self._value

    class DummyCombo:
        def __init__(self, value):
            self._value = value

        def currentData(self):
            return self._value

    class DummyPose:
        def __init__(self, x, y, z):
            self.position = type("Position", (), {"x": x, "y": y, "z": z})()
            self.orientation = type("Orientation", (), {"w": 1.0})()

    class DummySize:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class DummyObstacle:
        def __init__(self, obstacle_id, x, y, sx, sy, z=60.0, sz=120.0):
            self.id = obstacle_id
            self.enabled = True
            self.shape = "box"
            self.pose = DummyPose(x, y, z)
            self.size = DummySize(sx, sy, sz)

    print("MODULE_FILE", plugin.__file__)

    widget = SwarmControlWidget.__new__(SwarmControlWidget)
    widget._obstacles = {
        "obstacle_01": DummyObstacle("obstacle_01", 130.0, 55.0, 55.0, 70.0),
        "obstacle_02": DummyObstacle("obstacle_02", 130.0, -55.0, 55.0, 70.0),
    }
    widget._route_mode = "direct"
    widget._heading = DummyValue(90.0)
    widget._spacing = DummyValue(35.0)
    widget._formation_type = DummyCombo("line")

    mission_points = [
        {"x": 0.0, "y": 0.0, "z": 80.0},
        {"x": 260.0, "y": 0.0, "z": 80.0},
    ]
    route = widget._plan_center_route_preserving_formation(mission_points, "line", 35.0)

    print("ROUTE_MODE", widget._route_mode)
    print("ROUTE_POINTS", len(route))
    for index, point in enumerate(route):
        print("P{} {:.1f} {:.1f} {:.1f}".format(index + 1, point["x"], point["y"], point["z"]))

    samples = []
    for vehicle_id in VEHICLE_IDS:
        vehicle_route = widget._offset_route_for_vehicle(route, "line", 35.0, vehicle_id)
        mid_index = min(len(vehicle_route) // 2, max(len(vehicle_route) - 1, 0))
        mid_point = vehicle_route[mid_index]
        samples.append((vehicle_id, mid_point["x"], mid_point["y"]))
    print("MID_SAMPLES")
    for vehicle_id, x_value, y_value in samples:
        print(vehicle_id, round(x_value, 1), round(y_value, 1))

    print("REGROUP_POINTS", len(widget._regroup_points))
    for index, point in enumerate(widget._regroup_points):
        print("R{} {:.1f} {:.1f} {:.1f}".format(index + 1, point["x"], point["y"], point["z"]))

    base_offsets = compute_formation_offsets("line", 35.0, VEHICLE_IDS)
    print("BASE_LATERAL_SPAN", max(abs(offset[1]) for offset in base_offsets))
    compressed = False
    single_file = False
    for index in range(len(route)):
        profile = widget._corridor_profile_for_point(route, index, "line", 35.0)
        if profile["scale"] < 0.98:
            compressed = True
        if profile["single_file"]:
            single_file = True
        print(
            "PROFILE",
            index,
            "scale={:.3f}".format(profile["scale"]),
            "shift={:.1f}".format(profile["shift"]),
            "single_file={}".format(profile["single_file"]),
        )
    print("COMPRESSED", compressed)
    print("SINGLE_FILE", single_file)


if __name__ == "__main__":
    run()
