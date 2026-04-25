from pathlib import Path


TARGET = Path(r"D:\catkin_ws\src\mission_ui\src\mission_ui\swarm_control_plugin.py")


def main():
    text = TARGET.read_text(encoding="utf-8")
    patterns = [
        "def _publish_obstacle",
        "def _handle_obstacle_update",
        "def _update_route_preview",
        "def _plan_center_route_preserving_formation",
        "def _plan_task_points_with_avoidance",
        "def _build_detour_points",
        "def _upload_formation_mission_worker",
        "def _update_workflow_status",
        "_obstacle_counter",
        "obstacle_",
        "self._obstacles[msg.id]",
        "_route_mode",
    ]
    for pattern in patterns:
        index = text.find(pattern)
        print(f"=== {pattern} @ {index} ===")
        if index >= 0:
            print(text[index:index + 2200])
        print()


if __name__ == "__main__":
    main()
