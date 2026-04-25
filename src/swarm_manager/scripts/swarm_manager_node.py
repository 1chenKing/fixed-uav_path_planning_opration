#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from swarm_msgs.msg import FormationCommand, SwarmStatus


DEFAULT_VEHICLE_IDS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]


class SwarmManagerNode:
    def __init__(self):
        self.status_pub = rospy.Publisher("/swarm/status", SwarmStatus, queue_size=10)
        self.formation_sub = rospy.Subscriber(
            "/swarm/formation_cmd", FormationCommand, self.on_formation_cmd
        )
        self.safe_formation_sub = rospy.Subscriber(
            "/swarm/formation_cmd_safe", FormationCommand, self.on_safe_formation_cmd
        )
        self.phase_sub = rospy.Subscriber("/swarm/mission_phase_cmd", String, self.on_phase_cmd)
        self.active_formation = "line"
        self.vehicle_ids = list(DEFAULT_VEHICLE_IDS)
        self.last_command_time = None
        self.mission_phase = "idle"
        self.active_phases = {"arming", "armed", "mission_starting", "mission_active", "mission_refreshing", "holding"}

    def on_formation_cmd(self, msg):
        self.vehicle_ids = list(msg.vehicle_ids) if msg.vehicle_ids else list(DEFAULT_VEHICLE_IDS)
        self.last_command_time = rospy.Time.now()
        rospy.loginfo("Received desired formation command: %s", msg.formation_type or self.active_formation)

    def on_safe_formation_cmd(self, msg):
        self.active_formation = msg.formation_type or self.active_formation
        self.vehicle_ids = list(msg.vehicle_ids) if msg.vehicle_ids else list(DEFAULT_VEHICLE_IDS)
        self.last_command_time = rospy.Time.now()
        if self.mission_phase not in self.active_phases:
            self.mission_phase = "formation_ready"
        rospy.loginfo("Applied safe formation command: %s", self.active_formation)

    def on_phase_cmd(self, msg):
        self.mission_phase = msg.data or self.mission_phase
        rospy.loginfo("Mission phase updated: %s", self.mission_phase)

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            status = SwarmStatus()
            status.state = self.mission_phase or "idle"
            status.active_formation = self.active_formation
            status.vehicle_ids = self.vehicle_ids
            self.status_pub.publish(status)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("swarm_manager")
    SwarmManagerNode().spin()
