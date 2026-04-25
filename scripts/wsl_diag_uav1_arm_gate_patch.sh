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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_arm_gate_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_arm_gate_ros.log 2>&1 &
ROS_PID=$!

sleep 25

python3 - <<'PY'
import rospy
from mavros_msgs.msg import State, StatusText, ParamValue
from mavros_msgs.srv import ParamSet, CommandBool, SetMode

rospy.init_node("uav1_arm_gate_patch_diag", anonymous=True)
status_texts = []

def on_status(msg):
    if len(status_texts) < 30:
        status_texts.append((msg.severity, msg.text))

sub = rospy.Subscriber("/uav_1/mavros/statustext/recv", StatusText, on_status, queue_size=30)

rospy.wait_for_service("/uav_1/mavros/param/set", timeout=15.0)
rospy.wait_for_service("/uav_1/mavros/cmd/arming", timeout=15.0)
rospy.wait_for_service("/uav_1/mavros/set_mode", timeout=15.0)

param_set = rospy.ServiceProxy("/uav_1/mavros/param/set", ParamSet)
arming = rospy.ServiceProxy("/uav_1/mavros/cmd/arming", CommandBool)
set_mode = rospy.ServiceProxy("/uav_1/mavros/set_mode", SetMode)

def set_int(name, value_int):
    value = ParamValue()
    value.integer = int(value_int)
    value.real = 0.0
    return param_set(name, value)

print("STATE_INIT", rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0), flush=True)
print("SET_SYS_HAS_NUM_ASPD", set_int("SYS_HAS_NUM_ASPD", 0), flush=True)
print("SET_CBRK_SUPPLY_CHK", set_int("CBRK_SUPPLY_CHK", 894281), flush=True)
print("SET_NAV_DLL_ACT", set_int("NAV_DLL_ACT", 0), flush=True)
rospy.sleep(1.0)
print("SET_MODE_AUTO_LOITER", set_mode(0, "AUTO.LOITER"), flush=True)
rospy.sleep(1.0)
print("ARM_RESULT", arming(True), flush=True)
rospy.sleep(3.0)
print("STATE_AFTER", rospy.wait_for_message("/uav_1/mavros/state", State, timeout=10.0), flush=True)
print("STATUS_TEXTS", status_texts, flush=True)
sub.unregister()
PY

echo "=== INSTANCE 0 OUT LOG ==="
tail -n 120 /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/0/out.log || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
