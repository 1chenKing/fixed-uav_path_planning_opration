# PX4 Fixed-Wing Swarm Simulation Workspace

This repository is a starter workspace for a WSL2-based project targeting:

- Ubuntu 20.04
- ROS Noetic
- PX4 SITL
- Gazebo Classic
- Fixed-wing multi-UAV formation flight
- Obstacle-aware planning and avoidance
- A ROS-integrated GUI for mission editing and swarm control

## Goals

The project is structured to support:

1. Multi-vehicle fixed-wing simulation in Gazebo.
2. Formation management with dynamic shape switching.
3. Waypoint assignment for individual vehicles or the whole swarm.
4. Obstacle definition from a GUI and live rendering in Gazebo.
5. Real-time telemetry, command, and mission monitoring.

## Recommended stack

- PX4-Autopilot: SITL, MAVLink, fixed-wing airframes
- Gazebo Classic 11: native PX4 SITL support on Ubuntu 20.04
- MAVROS: ROS <-> PX4 bridge
- ROS packages in this repository: mission UI, formation manager, obstacle manager, swarm coordination
- Qt / rqt plugin for the integrated GUI

## Repository layout

```text
.
|-- docs/
|   `-- architecture.md
|-- scripts/
|   `-- bootstrap_wsl.sh
`-- src/
    |-- swarm_msgs/
    |-- swarm_manager/
    |-- formation_controller/
    |-- obstacle_manager/
    `-- mission_ui/
```

## High-level workflow

1. Install PX4, Gazebo, and ROS dependencies inside WSL2 Ubuntu 20.04.
2. Build the catkin workspace.
3. Launch PX4 multi-vehicle SITL.
4. Start ROS bridge nodes and the mission UI.
5. Send formation, waypoint, and obstacle commands from the GUI.
6. Visualize state in Gazebo and RViz.

## Current status

This repository currently provides:

- A project structure ready for ROS Noetic.
- Interface definitions for formation and obstacle commands.
- Launch and script placeholders to guide implementation.

It does not yet include:

- PX4 source checkout
- Full SITL launch implementation
- Actual formation control logic
- GUI implementation details

## Next implementation milestones

1. Add PX4 as an external dependency in WSL2.
2. Define the fixed-wing control mode transition rules.
3. Implement formation generation and swarm mission allocation.
4. Add obstacle-to-path-planner integration.
5. Build a Qt-based ground control panel.

