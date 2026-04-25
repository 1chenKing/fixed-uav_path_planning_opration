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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_uav1_quick_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_uav1_quick_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import math
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, StatusText
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool, CommandLong


def log(*parts):
    print(*parts, flush=True)


def offset_fix(latitude_deg, longitude_deg, east_m, north_m):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(latitude_deg)), 1.0)
    return latitude_deg + north_m / lat_scale, longitude_deg + east_m / lon_scale


rospy.init_node("uav1_quick_check", anonymous=True)
status_texts = []


def on_status(msg):
    if len(status_texts) < 30:
        status_texts.append((msg.severity, msg.text))


sub = rospy.Subscriber("/uav_1/mavros/statustext/recv", StatusText, on_status, queue_size=30)

try:
    state = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=20.0)
    fix = rospy.wait_for_message("/uav_1/mavros/global_position/global", NavSatFix, timeout=20.0)
    pose = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=20.0)
except Exception as exc:
    log("WAIT_MESSAGE_ERROR", repr(exc))
    sys.exit(2)

log("STATE_BEFORE", state.connected, state.armed, state.mode, state.system_status)
log("FIX", fix.latitude, fix.longitude, fix.altitude)
log("POSE", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

for service_name in [
    "/uav_1/mavros/mission/clear",
    "/uav_1/mavros/mission/push",
    "/uav_1/mavros/cmd/arming",
    "/uav_1/mavros/set_mode",
    "/uav_1/mavros/cmd/command",
]:
    try:
        rospy.wait_for_service(service_name, timeout=10.0)
        log("SERVICE_OK", service_name)
    except Exception as exc:
        log("SERVICE_WAIT_ERROR", service_name, repr(exc))
        sys.exit(3)

mission_clear = rospy.ServiceProxy("/uav_1/mavros/mission/clear", WaypointClear)
mission_push = rospy.ServiceProxy("/uav_1/mavros/mission/push", WaypointPush)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
command_long = rospy.ServiceProxy("/uav_1/mavros/cmd/command", CommandLong)

takeoff_lat, takeoff_lon = offset_fix(fix.latitude, fix.longitude, 25.0, 0.0)
wp_lat, wp_lon = offset_fix(fix.latitude, fix.longitude, 120.0, 20.0)

takeoff = Waypoint()
takeoff.frame = 3
takeoff.command = 22
takeoff.is_current = True
takeoff.autocontinue = True
takeoff.param1 = 15.0
takeoff.param2 = 15.0
takeoff.param3 = 0.0
takeoff.param4 = float("nan")
takeoff.x_lat = takeoff_lat
takeoff.y_long = takeoff_lon
takeoff.z_alt = 60.0

wp = Waypoint()
wp.frame = 3
wp.command = 16
wp.is_current = False
wp.autocontinue = True
wp.param1 = 0.0
wp.param2 = 15.0
wp.param3 = 0.0
wp.param4 = float("nan")
wp.x_lat = wp_lat
wp.y_long = wp_lon
wp.z_alt = 70.0

try:
    clear_resp = mission_clear()
    log("MISSION_CLEAR", clear_resp)
except Exception as exc:
    log("MISSION_CLEAR_ERROR", repr(exc))

try:
    push_resp = mission_push(0, [takeoff, wp])
    log("MISSION_PUSH", push_resp)
except Exception as exc:
    log("MISSION_PUSH_ERROR", repr(exc))

try:
    arm_resp = arming(True)
    log("ARM", arm_resp)
except Exception as exc:
    log("ARM_ERROR", repr(exc))

try:
    mode_resp = set_mode(0, "AUTO.MISSION")
    log("SET_MODE_AUTO_MISSION", mode_resp)
except Exception as exc:
    log("SET_MODE_ERROR", repr(exc))

try:
    start_resp = command_long(False, 300, 0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    log("MISSION_START", start_resp)
except Exception as exc:
    log("MISSION_START_ERROR", repr(exc))

rospy.sleep(5.0)
try:
    state_after = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0)
    pose_after = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=10.0)
    log("STATE_AFTER", state_after.connected, state_after.armed, state_after.mode, state_after.system_status)
    log("POSE_AFTER", pose_after.pose.position.x, pose_after.pose.position.y, pose_after.pose.position.z)
except Exception as exc:
    log("AFTER_WAIT_ERROR", repr(exc))

log("STATUS_TEXTS", status_texts)
sub.unregister()
PY

echo "=== ROS LOG TAIL ==="
tail -n 120 /tmp/codex_uav1_quick_ros.log || true

echo "=== FW LOG TAIL ==="
tail -n 80 /tmp/codex_uav1_quick_fw.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
