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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_uav1_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_uav1_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, StatusText
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool, CommandLong

rospy.init_node("uav1_mission_diag", anonymous=True)

status_texts = []

def _status_cb(msg):
    if len(status_texts) < 20:
        status_texts.append((msg.severity, msg.text))

status_sub = rospy.Subscriber("/uav_1/mavros/statustext/recv", StatusText, _status_cb, queue_size=20)

state = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=15.0)
fix = rospy.wait_for_message("/uav_1/mavros/global_position/global", NavSatFix, timeout=15.0)
pose = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=15.0)

print("STATE_BEFORE", state)
print("FIX", fix.latitude, fix.longitude, fix.altitude)
print("POSE", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

rospy.wait_for_service("/uav_1/mavros/mission/clear", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/mission/push", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/cmd/arming", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/set_mode", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/cmd/command", timeout=10.0)

mission_clear = rospy.ServiceProxy("/uav_1/mavros/mission/clear", WaypointClear)
mission_push = rospy.ServiceProxy("/uav_1/mavros/mission/push", WaypointPush)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
command_long = rospy.ServiceProxy("/uav_1/mavros/cmd/command", CommandLong)

def offset_fix(latitude_deg, longitude_deg, east_m, north_m):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(latitude_deg)), 1.0)
    return latitude_deg + north_m / lat_scale, longitude_deg + east_m / lon_scale

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

print("MISSION_CLEAR", mission_clear())
print("MISSION_PUSH", mission_push(0, [takeoff, wp]))
print("ARM", arming(True))
print("SET_MODE_AUTO_MISSION", set_mode(0, "AUTO.MISSION"))
print("MISSION_START", command_long(False, 300, 0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0))

rospy.sleep(8.0)
state_after = rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0)
pose_after = rospy.wait_for_message("/uav_1/mavros/local_position/pose", PoseStamped, timeout=10.0)
print("STATE_AFTER", state_after)
print("POSE_AFTER", pose_after.pose.position.x, pose_after.pose.position.y, pose_after.pose.position.z)
print("STATUS_TEXTS", status_texts)
status_sub.unregister()
PY

echo "=== ROS LOG TAIL ==="
tail -n 120 /tmp/codex_uav1_ros.log || true
echo "=== FW LOG TAIL ==="
tail -n 80 /tmp/codex_uav1_fw.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
