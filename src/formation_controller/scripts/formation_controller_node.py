#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from swarm_msgs.msg import FormationCommand
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_VEHICLE_IDS = ["uav_1", "uav_2", "uav_3", "uav_4", "uav_5", "uav_6"]


def compute_formation_positions(formation_type, spacing, heading_deg, anchor_xy, vehicle_ids):
    heading_rad = math.radians(heading_deg)
    forward = (math.cos(heading_rad), math.sin(heading_rad))
    left = (-forward[1], forward[0])
    anchor_x, anchor_y = anchor_xy
    offsets = []
    for index, _vehicle_id in enumerate(vehicle_ids):
        if formation_type == "column":
            offset = (-spacing * index, 0.0)
        elif formation_type == "v_shape":
            step = index // 2 + 1
            side = -1.0 if index % 2 else 1.0
            offset = (0.0, 0.0) if index == 0 else (-spacing * step, side * spacing * step)
        elif formation_type == "echelon_left":
            offset = (-spacing * index, spacing * index)
        elif formation_type == "echelon_right":
            offset = (-spacing * index, -spacing * index)
        else:
            offset = (0.0, spacing * (index - (len(vehicle_ids) - 1) / 2.0))
        offsets.append(offset)

    positions = {}
    for vehicle_id, (forward_offset, left_offset) in zip(vehicle_ids, offsets):
        x_value = anchor_x + forward[0] * forward_offset + left[0] * left_offset
        y_value = anchor_y + forward[1] * forward_offset + left[1] * left_offset
        positions[vehicle_id] = (x_value, y_value)
    return positions


def formation_model_name(vehicle_id):
    return "swarm_target_{}".format(vehicle_id)


def build_target_sdf(model_name):
    return """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='body'>
      <visual name='visual'>
        <geometry><sphere><radius>2.2</radius></sphere></geometry>
        <material>
          <ambient>0.10 0.85 0.65 0.85</ambient>
          <diffuse>0.10 0.85 0.65 0.85</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""".format(name=model_name)


class FormationControllerNode:
    def __init__(self):
        self.target_pub = rospy.Publisher("/swarm/debug_anchor", PoseStamped, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher("/swarm/formation_markers", MarkerArray, queue_size=10, latch=True)
        self.gazebo_status_pub = rospy.Publisher("/swarm/gazebo_sync_status", String, queue_size=10, latch=True)
        self.formation_sub = rospy.Subscriber("/swarm/formation_cmd_safe", FormationCommand, self.on_safe_formation_cmd)
        self.raw_formation_sub = rospy.Subscriber("/swarm/formation_cmd", FormationCommand, self.on_formation_cmd)
        self.phase_sub = rospy.Subscriber("/swarm/mission_phase_cmd", String, self.on_phase_cmd)
        self.setpoint_publishers = {
            vehicle_id: rospy.Publisher(
                "/{}/mavros/setpoint_position/local".format(vehicle_id), PoseStamped, queue_size=10
            )
            for vehicle_id in DEFAULT_VEHICLE_IDS
        }
        self.pose_subscribers = []
        self.current_positions = {}
        self.default_altitude = rospy.get_param("~default_altitude", 60.0)
        self.target_positions = {}
        self.last_command = self._build_default_command()
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 10.0)
        self.enable_setpoint_stream = rospy.get_param("~enable_setpoint_stream", False)
        self.enable_gazebo_sync = rospy.get_param("~enable_gazebo_sync", True)
        self.spawned_models = set()
        self.spawn_srv = None
        self.delete_srv = None
        self.set_state_srv = None
        self.mission_phase = "idle"
        for vehicle_id in DEFAULT_VEHICLE_IDS:
            self.pose_subscribers.append(
                rospy.Subscriber(
                    "/{}/mavros/local_position/pose".format(vehicle_id),
                    PoseStamped,
                    self._make_pose_callback(vehicle_id),
                    queue_size=1,
                )
            )
        if self.enable_gazebo_sync:
            self._init_gazebo_services()
        self._apply_command(self.last_command)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 2.0)), self.publish_vehicle_setpoints)

    def _build_default_command(self):
        msg = FormationCommand()
        msg.formation_type = "line"
        msg.spacing = 35.0
        msg.heading_deg = 90.0
        msg.vehicle_ids = list(DEFAULT_VEHICLE_IDS)
        msg.anchor.header.frame_id = "map"
        msg.anchor.pose.position.z = self.default_altitude
        msg.anchor.pose.orientation.w = 1.0
        return msg

    def _init_gazebo_services(self):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=2.0)
            rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            rospy.loginfo("Formation controller connected to Gazebo services.")
            self.gazebo_status_pub.publish(String(data="formation:connected"))
        except (rospy.ROSException, rospy.ServiceException):
            self.spawn_srv = None
            self.delete_srv = None
            self.set_state_srv = None
            rospy.logwarn_throttle(5.0, "Gazebo services unavailable, formation sync will retry.")
            self.gazebo_status_pub.publish(String(data="formation:unavailable"))

    def on_safe_formation_cmd(self, msg):
        rospy.loginfo(
            "Safe formation=%s spacing=%.2f heading=%.2f deg",
            msg.formation_type,
            msg.spacing,
            msg.heading_deg,
        )
        self._apply_command(msg)

    def on_formation_cmd(self, msg):
        rospy.loginfo(
            "Formation=%s spacing=%.2f heading=%.2f deg",
            msg.formation_type,
            msg.spacing,
            msg.heading_deg,
        )
        self._apply_command(msg)

    def on_phase_cmd(self, msg):
        self.mission_phase = msg.data or self.mission_phase

    def _apply_command(self, msg):
        vehicle_ids = list(msg.vehicle_ids) if msg.vehicle_ids else list(DEFAULT_VEHICLE_IDS)
        positions = compute_formation_positions(
            msg.formation_type,
            msg.spacing,
            msg.heading_deg,
            (msg.anchor.pose.position.x, msg.anchor.pose.position.y),
            vehicle_ids,
        )
        self.last_command = msg
        self.target_positions = positions
        self.publish_markers(positions, msg.anchor.pose.position.z)
        self.sync_gazebo_models(positions, msg.anchor.pose.position.z)
        self.target_pub.publish(msg.anchor)

    def _make_pose_callback(self, vehicle_id):
        def _callback(msg):
            self.current_positions[vehicle_id] = msg

        return _callback

    def publish_markers(self, positions, altitude):
        marker_array = MarkerArray()
        for index, (vehicle_id, (x_value, y_value)) in enumerate(positions.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "swarm_formation_targets"
            marker.id = index
            marker.action = Marker.ADD
            marker.type = Marker.SPHERE
            marker.pose.position.x = x_value
            marker.pose.position.y = y_value
            marker.pose.position.z = altitude
            marker.pose.orientation.w = 1.0
            marker.scale.x = 4.0
            marker.scale.y = 4.0
            marker.scale.z = 4.0
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 0.65
            marker.color.a = 0.85
            marker.text = vehicle_id
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def sync_gazebo_models(self, positions, altitude):
        if not self.enable_gazebo_sync:
            return
        if self.spawn_srv is None or self.delete_srv is None or self.set_state_srv is None:
            self._init_gazebo_services()
            if self.spawn_srv is None:
                return
        active_names = set()
        for vehicle_id, (x_value, y_value) in positions.items():
            model_name = formation_model_name(vehicle_id)
            active_names.add(model_name)
            try:
                if model_name not in self.spawned_models:
                    self.spawn_srv(model_name, build_target_sdf(model_name), "", PoseStamped().pose, "world")
                    self.spawned_models.add(model_name)
                state = ModelState()
                state.model_name = model_name
                state.pose.position.x = x_value
                state.pose.position.y = y_value
                state.pose.position.z = altitude
                state.pose.orientation.w = 1.0
                state.reference_frame = "world"
                self.set_state_srv(state)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(2.0, "Gazebo formation sync failed: %s", exc)
                self.gazebo_status_pub.publish(String(data="formation:error"))
        for model_name in list(self.spawned_models - active_names):
            try:
                self.delete_srv(model_name)
            except rospy.ServiceException:
                pass
            self.spawned_models.discard(model_name)

    def publish_vehicle_setpoints(self, _event):
        if not self.enable_setpoint_stream:
            return
        if self.mission_phase in {"mission_uploaded", "arming", "armed", "mission_starting", "mission_active", "holding"}:
            return
        if self.last_command is None or not self.target_positions:
            return
        altitude = self.last_command.anchor.pose.position.z
        heading_rad = math.radians(self.last_command.heading_deg)
        sin_half = math.sin(heading_rad / 2.0)
        cos_half = math.cos(heading_rad / 2.0)
        stamp = rospy.Time.now()
        for vehicle_id, (x_value, y_value) in self.target_positions.items():
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "map"
            msg.pose.position.x = x_value
            msg.pose.position.y = y_value
            msg.pose.position.z = altitude
            msg.pose.orientation.z = sin_half
            msg.pose.orientation.w = cos_half
            self.setpoint_publishers[vehicle_id].publish(msg)


if __name__ == "__main__":
    rospy.init_node("formation_controller")
    FormationControllerNode()
    rospy.spin()
