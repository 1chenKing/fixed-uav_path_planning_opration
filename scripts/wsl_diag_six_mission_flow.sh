#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

cleanup() {
  pkill -f "roslaunch swarm_bringup swarm_multi_uav_6.launch" || true
  pkill -f "sitl_multiple_run.sh -m plane -n 6 -w empty" || true
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x gazebo || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_six_flow_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_six_flow_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, ParamValue
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool, CommandLong, ParamSet

VEHICLE_IDS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]

def compute_positions(spacing, heading_deg, anchor_xy, vehicle_ids):
    heading_rad = math.radians(heading_deg)
    forward = (math.cos(heading_rad), math.sin(heading_rad))
    left = (-forward[1], forward[0])
    anchor_x, anchor_y = anchor_xy
    positions = {}
    for index, vehicle_id in enumerate(vehicle_ids):
        left_offset = spacing * (index - (len(vehicle_ids) - 1) / 2.0)
        x_value = anchor_x + left[0] * left_offset
        y_value = anchor_y + left[1] * left_offset
        positions[vehicle_id] = (x_value, y_value)
    return positions, forward

def offset_fix(latitude_deg, longitude_deg, east_m, north_m):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(latitude_deg)), 1.0)
    return latitude_deg + north_m / lat_scale, longitude_deg + east_m / lon_scale

def make_wp(command, lat, lon, alt, is_current, param1=0.0):
    wp = Waypoint()
    wp.frame = 3
    wp.command = command
    wp.is_current = is_current
    wp.autocontinue = True
    wp.param1 = param1
    wp.param2 = 15.0
    wp.param3 = 0.0
    wp.param4 = float("nan")
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    return wp

rospy.init_node("six_mission_flow_diag", anonymous=True)

spacing = 35.0
heading_deg = 90.0
anchor_xy = (120.0, 0.0)
target_alt = 70.0
landing_alt = 10.0
positions, forward = compute_positions(spacing, heading_deg, anchor_xy, VEHICLE_IDS)
glide_delta = max(target_alt - landing_alt, 5.0)
landing_distance = max(glide_delta / math.tan(math.radians(8.0)), 450.0)

summary = []

for vehicle_id in VEHICLE_IDS:
    state = rospy.wait_for_message(f"/{vehicle_id}/mavros/state", State, timeout=20.0)
    fix = rospy.wait_for_message(f"/{vehicle_id}/mavros/global_position/global", NavSatFix, timeout=20.0)
    pose = rospy.wait_for_message(f"/{vehicle_id}/mavros/local_position/pose", PoseStamped, timeout=20.0)

    for service_name in [
        f"/{vehicle_id}/mavros/param/set",
        f"/{vehicle_id}/mavros/mission/clear",
        f"/{vehicle_id}/mavros/mission/push",
        f"/{vehicle_id}/mavros/cmd/arming",
        f"/{vehicle_id}/mavros/set_mode",
        f"/{vehicle_id}/mavros/cmd/command",
    ]:
        rospy.wait_for_service(service_name, timeout=15.0)

    param_set = rospy.ServiceProxy(f"/{vehicle_id}/mavros/param/set", ParamSet)
    mission_clear = rospy.ServiceProxy(f"/{vehicle_id}/mavros/mission/clear", WaypointClear)
    mission_push = rospy.ServiceProxy(f"/{vehicle_id}/mavros/mission/push", WaypointPush)
    arming = rospy.ServiceProxy(f"/{vehicle_id}/mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy(f"/{vehicle_id}/mavros/set_mode", SetMode)
    command_long = rospy.ServiceProxy(f"/{vehicle_id}/mavros/cmd/command", CommandLong)

    def set_int(name, value_int):
        value = ParamValue()
        value.integer = int(value_int)
        value.real = 0.0
        return param_set(name, value)

    print(vehicle_id, "STATE_BEFORE", state.armed, state.mode, flush=True)
    print(vehicle_id, "SET_SYS_HAS_NUM_ASPD", set_int("SYS_HAS_NUM_ASPD", 0), flush=True)
    print(vehicle_id, "SET_CBRK_SUPPLY_CHK", set_int("CBRK_SUPPLY_CHK", 894281), flush=True)
    print(vehicle_id, "SET_NAV_DLL_ACT", set_int("NAV_DLL_ACT", 0), flush=True)

    target_x, target_y = positions[vehicle_id]
    takeoff_lat, takeoff_lon = offset_fix(fix.latitude, fix.longitude, 25.0, 0.0)
    wp_lat, wp_lon = offset_fix(fix.latitude, fix.longitude, target_x - pose.pose.position.x, target_y - pose.pose.position.y)
    land_target_x = target_x + forward[0] * landing_distance
    land_target_y = target_y + forward[1] * landing_distance
    land_lat, land_lon = offset_fix(fix.latitude, fix.longitude, land_target_x - pose.pose.position.x, land_target_y - pose.pose.position.y)

    mission = [
        make_wp(22, takeoff_lat, takeoff_lon, 60.0, True, 15.0),
        make_wp(16, wp_lat, wp_lon, target_alt, False, 0.0),
        make_wp(21, land_lat, land_lon, landing_alt, False, 0.0),
    ]

    clear_resp = mission_clear()
    push_resp = mission_push(0, mission)
    arm_resp = arming(True)
    mode_resp = set_mode(0, "AUTO.MISSION")
    start_resp = command_long(False, 300, 0, 0.0, float(len(mission) - 1), 0.0, 0.0, 0.0, 0.0, 0.0)
    summary.append((vehicle_id, clear_resp.success, push_resp.success, push_resp.wp_transfered, arm_resp.success, arm_resp.result, mode_resp.mode_sent, start_resp.success, start_resp.result))

rospy.sleep(10.0)
for vehicle_id in VEHICLE_IDS:
    pose_after = rospy.wait_for_message(f"/{vehicle_id}/mavros/local_position/pose", PoseStamped, timeout=10.0)
    state_after = rospy.wait_for_message(f"/{vehicle_id}/mavros/state", State, timeout=10.0)
    print(vehicle_id, "STATE_AFTER", state_after.armed, state_after.mode, state_after.system_status, flush=True)
    print(vehicle_id, "POSE_AFTER", pose_after.pose.position.x, pose_after.pose.position.y, pose_after.pose.position.z, flush=True)

print("SUMMARY", summary, flush=True)
PY

echo "=== INSTANCE 0 OUT LOG ==="
tail -n 80 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/0/out.log || true

echo "=== INSTANCE 5 OUT LOG ==="
tail -n 80 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/5/out.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
