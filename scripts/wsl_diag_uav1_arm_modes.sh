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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_arm_modes_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_arm_modes_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

rospy.init_node("uav1_arm_modes_diag", anonymous=True)

rospy.wait_for_service("/uav_1/mavros/set_mode", timeout=10.0)
rospy.wait_for_service("/uav_1/mavros/cmd/arming", timeout=10.0)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)

print("STATE_INIT", rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0))
for mode in ["MANUAL", "STABILIZED", "AUTO.LOITER"]:
    try:
        print("SET_MODE", mode, set_mode(0, mode))
        rospy.sleep(1.5)
        print("STATE_NOW", rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0))
        print("ARM_RESULT", mode, arming(True))
        rospy.sleep(1.5)
        print("STATE_POST_ARM", rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0))
        arming(False)
        rospy.sleep(0.8)
    except Exception as exc:
        print("MODE_TEST_ERROR", mode, exc)
PY

echo "=== ROS LOG TAIL ==="
tail -n 120 /tmp/codex_arm_modes_ros.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
