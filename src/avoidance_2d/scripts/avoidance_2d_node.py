#!/usr/bin/env python3
import copy
import math

import rospy
from swarm_msgs.msg import FormationCommand, Obstacle, SwarmStatus


class Avoidance2DNode:
    def __init__(self):
        self._obstacles = {}
        self._safety_margin = rospy.get_param("~safety_margin", 18.0)
        self._formation_sub = rospy.Subscriber("/swarm/formation_cmd", FormationCommand, self._on_formation)
        self._obstacle_sub = rospy.Subscriber("/swarm/obstacles", Obstacle, self._on_obstacle)
        self._safe_pub = rospy.Publisher("/swarm/formation_cmd_safe", FormationCommand, queue_size=10, latch=True)
        self._status_pub = rospy.Publisher("/swarm/avoidance/state", SwarmStatus, queue_size=10, latch=True)

    def _on_obstacle(self, msg):
        self._obstacles[msg.id] = msg

    def _on_formation(self, msg):
        safe_msg = copy.deepcopy(msg)
        anchor_x = msg.anchor.pose.position.x
        anchor_y = msg.anchor.pose.position.y
        adjusted_x = anchor_x
        adjusted_y = anchor_y
        state = "clear"

        for obstacle in self._obstacles.values():
            if not obstacle.enabled:
                continue
            half_x = obstacle.size.x / 2.0 + self._safety_margin
            half_y = obstacle.size.y / 2.0 + self._safety_margin
            delta_x = adjusted_x - obstacle.pose.position.x
            delta_y = adjusted_y - obstacle.pose.position.y
            if abs(delta_x) < half_x and abs(delta_y) < half_y:
                if (half_x - abs(delta_x)) < (half_y - abs(delta_y)):
                    adjusted_x += math.copysign(half_x - abs(delta_x) + 5.0, delta_x if delta_x != 0.0 else 1.0)
                else:
                    adjusted_y += math.copysign(half_y - abs(delta_y) + 5.0, delta_y if delta_y != 0.0 else 1.0)
                state = "rerouted"

        safe_msg.anchor.pose.position.x = adjusted_x
        safe_msg.anchor.pose.position.y = adjusted_y
        safe_msg.anchor.header.stamp = rospy.Time.now()
        self._safe_pub.publish(safe_msg)

        status = SwarmStatus()
        status.header.stamp = rospy.Time.now()
        status.vehicle_ids = msg.vehicle_ids
        status.state = state
        status.active_formation = safe_msg.formation_type
        self._status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("avoidance_2d")
    Avoidance2DNode()
    rospy.spin()
