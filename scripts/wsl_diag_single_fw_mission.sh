#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

cleanup() {
  pkill -f "roslaunch swarm_bringup swarm_namespaced.launch" || true
  pkill -f "make px4_sitl gazebo-classic_plane" || true
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x gazebo || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
make px4_sitl gazebo-classic_plane >/tmp/single_fw_diag_px4.log 2>&1 &
PX4_PID=$!

sleep 20

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_namespaced.launch vehicle_ns:=uav_1 fcu_url:=udp://:14540@127.0.0.1:14580 tgt_system:=1 >/tmp/single_fw_diag_mavros.log 2>&1 &
MAVROS_PID=$!

sleep 15

python3 - <<'PY'
import math
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Waypoint, ExtendedState
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool

rospy.init_node("single_fw_diag", anonymous=True)

def wait_topic(topic, msg_type, timeout=10.0):
    return rospy.wait_for_message(topic, msg_type, timeout=timeout)

print("=== state ===")
state = wait_topic("/uav_1/mavros/state", State, timeout=15.0)
print(state)

print("=== global ===")
fix = wait_topic("/uav_1/mavros/global_position/global", NavSatFix, timeout=15.0)
print(fix)

print("=== local pose ===")
pose = wait_topic("/uav_1/mavros/local_position/pose", PoseStamped, timeout=15.0)
print(pose)

print("=== extended state ===")
ext_state = wait_topic("/uav_1/mavros/extended_state", ExtendedState, timeout=15.0)
print(ext_state)

rospy.wait_for_service("/uav_1/mavros/mission/clear", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/mission/push", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/set_mode", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/cmd/arming", timeout=10.0)

mission_clear = rospy.ServiceProxy("/uav_1/mavros/mission/clear", WaypointClear)
mission_push = rospy.ServiceProxy("/uav_1/mavros/mission/push", WaypointPush)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)

def offset_fix(lat_deg, lon_deg, east_m, north_m):
    lat_scale = 111320.0
    lon_scale = max(111320.0 * math.cos(math.radians(lat_deg)), 1.0)
    return lat_deg + north_m / lat_scale, lon_deg + east_m / lon_scale

takeoff_lat, takeoff_lon = offset_fix(fix.latitude, fix.longitude, 20.0, 0.0)
wp_lat, wp_lon = offset_fix(fix.latitude, fix.longitude, 120.0, 0.0)

takeoff = Waypoint()
takeoff.frame = 3
takeoff.command = 22
takeoff.is_current = True
takeoff.autocontinue = True
takeoff.param1 = 15.0
takeoff.param2 = 15.0
takeoff.param4 = float("nan")
takeoff.x_lat = takeoff_lat
takeoff.y_long = takeoff_lon
takeoff.z_alt = 60.0

wp = Waypoint()
wp.frame = 3
wp.command = 16
wp.is_current = False
wp.autocontinue = True
wp.param4 = float("nan")
wp.x_lat = wp_lat
wp.y_long = wp_lon
wp.z_alt = 60.0

print("=== clear mission ===")
print(mission_clear())
print("=== push mission ===")
print(mission_push(0, [takeoff, wp]))
print("=== arm ===")
print(arming(True))
print("=== set AUTO.MISSION ===")
print(set_mode(0, "AUTO.MISSION"))
PY

sleep 5

echo "=== single mavros log tail ==="
tail -n 80 /tmp/single_fw_diag_mavros.log || true
echo "=== single px4 log tail ==="
tail -n 80 /tmp/single_fw_diag_px4.log || true

kill "$MAVROS_PID" || true
kill "$PX4_PID" || true
cleanup
