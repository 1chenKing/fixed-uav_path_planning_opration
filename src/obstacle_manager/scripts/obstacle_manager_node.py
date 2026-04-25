#!/usr/bin/env python3
import os
import shutil
import subprocess
import tempfile

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from swarm_msgs.msg import Obstacle


def obstacle_model_name(obstacle_id):
    return "swarm_obstacle_{}".format(obstacle_id)


def build_obstacle_sdf(model_name, obstacle):
    geometry_tag = "<box><size>{:.3f} {:.3f} {:.3f}</size></box>".format(
        obstacle.size.x,
        obstacle.size.y,
        max(obstacle.size.z, 2.0),
    )
    if obstacle.shape == "cylinder":
        geometry_tag = "<cylinder><radius>{:.3f}</radius><length>{:.3f}</length></cylinder>".format(
            max(obstacle.size.x, obstacle.size.y) / 2.0,
            max(obstacle.size.z, 2.0),
        )
    return """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='body'>
      <visual name='visual'>
        <geometry>{geometry}</geometry>
        <material>
          <ambient>0.95 0.35 0.15 0.75</ambient>
          <diffuse>0.95 0.35 0.15 0.75</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>{geometry}</geometry>
      </collision>
    </link>
  </model>
</sdf>""".format(name=model_name, geometry=geometry_tag)


class ObstacleManagerNode:
    def __init__(self):
        self.markers_pub = rospy.Publisher("/swarm/obstacle_markers", MarkerArray, queue_size=10)
        self.gazebo_status_pub = rospy.Publisher("/swarm/gazebo_sync_status", String, queue_size=10, latch=True)
        self.obstacles = {}
        self.spawned_models = set()
        self.enable_gazebo_sync = rospy.get_param("~enable_gazebo_sync", True)
        self.spawn_srv = None
        self.delete_srv = None
        self.set_state_srv = None
        self.gz_cli_available = shutil.which("gz") is not None
        if self.enable_gazebo_sync:
            self._init_gazebo_services()
        rospy.Subscriber("/swarm/obstacles", Obstacle, self.on_obstacle)

    def _init_gazebo_services(self):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=2.0)
            rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            rospy.loginfo("Obstacle manager connected to Gazebo services.")
            self.gazebo_status_pub.publish(String(data="obstacle:connected"))
        except (rospy.ROSException, rospy.ServiceException):
            self.spawn_srv = None
            self.delete_srv = None
            self.set_state_srv = None
            if self.gz_cli_available:
                rospy.logwarn_throttle(5.0, "Gazebo ROS services unavailable, obstacle manager falling back to gz model.")
                self.gazebo_status_pub.publish(String(data="obstacle:connected"))
            else:
                rospy.logwarn_throttle(5.0, "Gazebo services unavailable, obstacle sync will retry.")
                self.gazebo_status_pub.publish(String(data="obstacle:unavailable"))

    def _run_gz_model(self, args, sdf_string=None):
        command = ["gz", "model"] + args
        if sdf_string is None:
            subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return
        with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False, encoding="utf-8") as sdf_file:
            sdf_file.write(sdf_string)
            temp_path = sdf_file.name
        try:
            subprocess.run(
                command + ["--spawn-file", temp_path],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        finally:
            try:
                os.unlink(temp_path)
            except OSError:
                pass

    def _sync_gazebo_model_cli(self, obstacle):
        model_name = obstacle_model_name(obstacle.id)
        if not obstacle.enabled:
            if model_name in self.spawned_models:
                try:
                    self._run_gz_model(["--model-name", model_name, "--delete"])
                except subprocess.CalledProcessError as exc:
                    rospy.logwarn_throttle(2.0, "gz obstacle delete failed: %s", exc.stderr.strip() if exc.stderr else exc)
                    self.gazebo_status_pub.publish(String(data="obstacle:error"))
                    return
                self.spawned_models.discard(model_name)
            return

        try:
            if model_name not in self.spawned_models:
                sdf = build_obstacle_sdf(model_name, obstacle)
                self._run_gz_model(
                    [
                        "--model-name",
                        model_name,
                        "--pose-x",
                        str(obstacle.pose.position.x),
                        "--pose-y",
                        str(obstacle.pose.position.y),
                        "--pose-z",
                        str(obstacle.pose.position.z),
                    ],
                    sdf_string=sdf,
                )
                self.spawned_models.add(model_name)
            else:
                self._run_gz_model(
                    [
                        "--model-name",
                        model_name,
                        "--pose-x",
                        str(obstacle.pose.position.x),
                        "--pose-y",
                        str(obstacle.pose.position.y),
                        "--pose-z",
                        str(obstacle.pose.position.z),
                        "--pose-R",
                        "0",
                        "--pose-P",
                        "0",
                        "--pose-Y",
                        "0",
                    ]
                )
            self.gazebo_status_pub.publish(String(data="obstacle:connected"))
        except subprocess.CalledProcessError as exc:
            rospy.logwarn_throttle(2.0, "gz obstacle sync failed: %s", exc.stderr.strip() if exc.stderr else exc)
            self.gazebo_status_pub.publish(String(data="obstacle:error"))

    def on_obstacle(self, msg):
        self.obstacles[msg.id] = msg
        self.publish_markers()
        self.sync_gazebo_model(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        for index, (obstacle_id, obstacle) in enumerate(self.obstacles.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "swarm_obstacles"
            marker.id = index
            marker.action = Marker.ADD if obstacle.enabled else Marker.DELETE
            marker.type = Marker.CYLINDER if obstacle.shape == "cylinder" else Marker.CUBE
            marker.pose = obstacle.pose
            marker.scale = obstacle.size
            marker.color.r = 1.0
            marker.color.g = 0.3
            marker.color.b = 0.1
            marker.color.a = 0.6
            marker.text = obstacle_id
            marker_array.markers.append(marker)
        self.markers_pub.publish(marker_array)

    def sync_gazebo_model(self, obstacle):
        if not self.enable_gazebo_sync:
            return
        if self.spawn_srv is None or self.delete_srv is None or self.set_state_srv is None:
            self._init_gazebo_services()
            if self.spawn_srv is None:
                if self.gz_cli_available:
                    self._sync_gazebo_model_cli(obstacle)
                return
        model_name = obstacle_model_name(obstacle.id)
        try:
            if not obstacle.enabled:
                if model_name in self.spawned_models:
                    self.delete_srv(model_name)
                    self.spawned_models.discard(model_name)
                return

            if model_name not in self.spawned_models:
                sdf = build_obstacle_sdf(model_name, obstacle)
                self.spawn_srv(model_name, sdf, "", obstacle.pose, "world")
                self.spawned_models.add(model_name)
            state = ModelState()
            state.model_name = model_name
            state.pose = obstacle.pose
            state.reference_frame = "world"
            self.set_state_srv(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Gazebo obstacle sync failed: %s", exc)
            self.gazebo_status_pub.publish(String(data="obstacle:error"))


if __name__ == "__main__":
    rospy.init_node("obstacle_manager")
    ObstacleManagerNode()
    rospy.spin()
