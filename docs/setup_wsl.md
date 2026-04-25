# WSL2 Setup Guide

## Target environment

- WSL2
- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11
- PX4 SITL

## 1. Prepare the workspace

Inside WSL2:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_init_workspace src
```

Copy or clone this repository into `~/catkin_ws`.

## 2. Install ROS and simulation dependencies

Run:

```bash
chmod +x scripts/bootstrap_wsl.sh
./scripts/bootstrap_wsl.sh
```

## 3. Clone PX4

Recommended location:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

After installation, restart the WSL shell.

## 4. Install MAVROS extras

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

## 5. Build the catkin workspace

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 6. First integration target

Before multi-UAV work, validate:

1. PX4 fixed-wing single-aircraft SITL launches correctly.
2. Gazebo world renders correctly.
3. MAVROS connects and publishes vehicle state into ROS.
4. `swarm_stack.launch` can run without package errors.

## 7. Multi-UAV direction

For fixed-wing swarm SITL, use one PX4 instance plus one MAVROS instance per aircraft namespace.

Suggested namespaces:

- `uav_1`
- `uav_2`
- `uav_3`

Each namespace should map:

- unique MAVLink UDP ports
- unique system IDs
- unique spawn pose in Gazebo

## 8. GUI direction

Recommended implementation order:

1. Build a simple `rqt` plugin to publish formation and obstacle commands.
2. Add RViz markers for waypoints and obstacles.
3. If richer interaction is needed, migrate to a standalone Qt panel that still talks to ROS topics/services.

## 9. Important engineering advice

- Start with 2 aircraft, not 5 or more.
- Start with waypoint-level coordination, not full offboard low-level control.
- Use a flat world first, then add terrain and clutter.
- Keep obstacle avoidance separate from formation generation at first.

