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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_mission_land_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_mission_land_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, StatusText, ParamValue
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool, CommandLong, ParamSet

rospy.init_node("uav1_mission_with_landing_diag", anonymous=True)
status_texts = []

def on_status(msg):
    if len(status_texts) < 30:
        status_texts.append((msg.severity, msg.text))

sub = rospy.Subscriber("/uav_1/mavros/statustext/recv", StatusText, on_status, queue_size=30)

def offset_fix(latitude_deg, longitude_deg, east_m, north_m):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(latitude_deg)), 1.0)
    return latitude_deg + north_m / lat_scale, longitude_deg + east_m / lon_scale

state = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=20.0)
fix = rospy.wait_for_message("/uav_1/mavros/global_position/global", NavSatFix, timeout=20.0)
pose = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=20.0)

print("STATE_BEFORE", state, flush=True)
print("POSE_BEFORE", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, flush=True)

for service_name in [
    "/uav_1/mavros/param/set",
    "/uav_1/mavros/mission/clear",
    "/uav_1/mavros/mission/push",
    "/uav_1/mavros/cmd/arming",
    "/uav_1/mavros/set_mode",
    "/uav_1/mavros/cmd/command",
]:
    rospy.wait_for_service(service_name, timeout=15.0)

param_set = rospy.ServiceProxy("/uav_1/mavros/param/set", ParamSet)
mission_clear = rospy.ServiceProxy("/uav_1/mavros/mission/clear", WaypointClear)
mission_push = rospy.ServiceProxy("/uav_1/mavros/mission/push", WaypointPush)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
command_long = rospy.ServiceProxy("/uav_1/mavros/cmd/command", CommandLong)

def set_int(name, value_int):
    value = ParamValue()
    value.integer = int(value_int)
    value.real = 0.0
    return param_set(name, value)

print("SET_SYS_HAS_NUM_ASPD", set_int("SYS_HAS_NUM_ASPD", 0), flush=True)
print("SET_CBRK_SUPPLY_CHK", set_int("CBRK_SUPPLY_CHK", 894281), flush=True)
print("SET_NAV_DLL_ACT", set_int("NAV_DLL_ACT", 0), flush=True)

takeoff_lat, takeoff_lon = offset_fix(fix.latitude, fix.longitude, 25.0, 0.0)
wp1_lat, wp1_lon = offset_fix(fix.latitude, fix.longitude, 120.0, 20.0)
landing_alt = 10.0
glide_delta = max(70.0 - landing_alt, 5.0)
landing_distance = max(glide_delta / math.tan(math.radians(8.0)), 450.0)
land_lat, land_lon = offset_fix(fix.latitude, fix.longitude, 120.0 + landing_distance, 20.0)

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

mission = [
    make_wp(22, takeoff_lat, takeoff_lon, 60.0, True, 15.0),
    make_wp(16, wp1_lat, wp1_lon, 70.0, False, 0.0),
    make_wp(21, land_lat, land_lon, landing_alt, False, 0.0),
]

print("MISSION_CLEAR", mission_clear(), flush=True)
print("MISSION_PUSH", mission_push(0, mission), flush=True)
print("ARM", arming(True), flush=True)
print("SET_MODE_AUTO_MISSION", set_mode(0, "AUTO.MISSION"), flush=True)
print("MISSION_START", command_long(False, 300, 0, 0.0, float(len(mission) - 1), 0.0, 0.0, 0.0, 0.0, 0.0), flush=True)

rospy.sleep(8.0)
state_after = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0)
pose_after = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=10.0)
print("STATE_AFTER", state_after, flush=True)
print("POSE_AFTER", pose_after.pose.position.x, pose_after.pose.position.y, pose_after.pose.position.z, flush=True)
print("STATUS_TEXTS", status_texts, flush=True)
sub.unregister()
PY

echo "=== INSTANCE 0 OUT LOG ==="
tail -n 160 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/0/out.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
