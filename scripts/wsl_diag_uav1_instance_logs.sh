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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_instance_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_instance_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import rospy
from mavros_msgs.srv import CommandBool, SetMode

rospy.init_node("uav1_instance_logs_diag", anonymous=True)
rospy.wait_for_service("/uav_1/mavros/cmd/arming", timeout=15.0)
rospy.wait_for_service("/uav_1/mavros/set_mode", timeout=15.0)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)
print("SET_MODE_AUTO_LOITER", set_mode(0, "AUTO.LOITER"), flush=True)
rospy.sleep(1.0)
print("ARM_RESULT", arming(True), flush=True)
rospy.sleep(3.0)
PY

echo "=== INSTANCE 0 OUT LOG ==="
tail -n 200 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/0/out.log || true

echo "=== INSTANCE 0 ERR LOG ==="
tail -n 200 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/0/err.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
