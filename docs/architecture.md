# System Architecture

## 1. Core design choice

For ROS Noetic + Ubuntu 20.04 + PX4 fixed-wing simulation, the most stable path is:

- PX4 SITL + Gazebo Classic
- MAVROS for vehicle communication
- ROS nodes for swarm logic
- Qt/rqt GUI for operations

This avoids introducing ROS 2 or newer Gazebo variants into the initial phase.

## 2. Module breakdown

### PX4 and simulation layer

- `PX4-Autopilot`
  - Runs one SITL instance per fixed-wing UAV.
  - Publishes state through MAVLink.
- `Gazebo`
  - Simulates vehicle physics and visual world.
  - Renders aircraft, obstacles, and the environment in real time.

### ROS bridge layer

- `MAVROS`
  - One MAVROS namespace per aircraft.
  - Handles topics/services for position, mode, arming, mission, and commands.

### Swarm logic layer

- `swarm_manager`
  - Maintains the swarm registry and per-vehicle state.
  - Coordinates takeoff, loiter, mission start, pause, and return.
- `formation_controller`
  - Computes relative geometry for line, V-shape, echelon, column, and custom formations.
  - Converts formation commands into per-aircraft target waypoints.
- `obstacle_manager`
  - Owns obstacle definitions from the GUI.
  - Publishes obstacle markers and planner constraints.
- `planner` (future)
  - Produces obstacle-aware paths for fixed-wing kinematics.
  - Should prefer smooth curvature constraints over multirotor-style stop-and-turn plans.

### Human-machine interface layer

- `mission_ui`
  - Graphical control panel.
  - Mission editing, formation switching, obstacle editing, and status display.
  - Can be implemented as:
    - an `rqt` plugin for fast ROS integration, or
    - a standalone Qt app for richer UX.

## 3. Recommended fixed-wing control strategy

Fixed-wing vehicles cannot hover or turn in place, so the control strategy should be:

1. The operator defines a swarm-level mission.
2. The formation controller derives reference trajectories for each aircraft.
3. A path smoothing layer respects minimum turn radius and airspeed constraints.
4. Each vehicle receives waypoint or setpoint sequences through MAVROS/PX4.

For the first project phase, use waypoint-based coordinated flight instead of low-level offboard body-rate control. It is simpler, safer, and more robust in SITL.

## 4. GUI feature map

The integrated GUI should include:

- Map or top-view mission canvas
- Swarm vehicle list
- Formation selector
- Waypoint editor
- Obstacle editor
- Start/pause/RTL controls
- Live telemetry panel
- Gazebo/RViz synchronized visualization

## 5. Suggested ROS interfaces

### Topics

- `/swarm/formation_cmd`
- `/swarm/global_mission`
- `/swarm/obstacles`
- `/swarm/status`

### Services

- `/swarm/set_formation`
- `/swarm/push_mission`
- `/swarm/spawn_obstacle`
- `/swarm/remove_obstacle`
- `/swarm/arm_all`
- `/swarm/start_all`

## 6. Namespacing plan

Each aircraft should live under a namespace:

- `/uav_1/...`
- `/uav_2/...`
- `/uav_3/...`

Each namespace should contain:

- MAVROS node
- vehicle status topics
- mission topics
- command services

## 7. Deployment phases

### Phase 1

- Single fixed-wing SITL validation
- GUI prototype
- Obstacle rendering only

### Phase 2

- Two to four fixed-wing vehicles
- Basic formation switching
- Shared waypoint mission push

### Phase 3

- Obstacle-aware trajectory generation
- Conflict handling between vehicles
- Multi-vehicle operator workflow polishing

