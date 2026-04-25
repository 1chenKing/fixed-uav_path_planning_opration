# PX4 Multi-UAV Bringup Plan

## Recommended first runnable target

The project target is 6 fixed-wing vehicles.

For engineering stability, bring-up should still follow:

1. validate 1 vehicle
2. validate 2 vehicles
3. scale to the final 6-vehicle formation setup

## Process layout per aircraft

Each aircraft should have:

- one PX4 SITL instance
- one Gazebo model spawn
- one MAVROS bridge
- one namespace
- one unique system ID
- one unique UDP port pair

## Namespace proposal

- `/uav_1`
- `/uav_2`
- `/uav_3`

## Ports proposal

Example mapping for the PX4 built-in `Tools/simulation/gazebo-classic/sitl_multiple_run.sh` helper:

- `uav_1`: PX4 simulator UDP port `14561`, MAVROS local bind `14540`, `tgt_system=2`
- `uav_2`: PX4 simulator UDP port `14562`, MAVROS local bind `14541`, `tgt_system=3`
- `uav_3`: PX4 simulator UDP port `14563`, MAVROS local bind `14542`, `tgt_system=4`
- `uav_4`: PX4 simulator UDP port `14564`, MAVROS local bind `14543`, `tgt_system=5`
- `uav_5`: PX4 simulator UDP port `14565`, MAVROS local bind `14544`, `tgt_system=6`
- `uav_6`: PX4 simulator UDP port `14566`, MAVROS local bind `14545`, `tgt_system=7`
- `uav_N`: `tgt_system` and UDP port must both remain unique

The exact values can be adjusted, but they must not conflict.

## Fixed-wing operation model

The first implementation should use:

- PX4 mission mode
- coordinated waypoint push
- formation geometry converted into per-aircraft waypoint offsets

Avoid low-level offboard rate control in the first phase.

## Obstacle handling path

1. Operator creates obstacles in GUI.
2. `obstacle_manager` publishes obstacle topics and markers.
3. Planner computes smooth avoidance corridors.
4. Formation controller updates per-aircraft path references.
5. Mission manager pushes revised waypoint sequences.

## GUI recommendation

For ROS Noetic, start with an `rqt` plugin and embed:

- formation selector
- waypoint list/table
- obstacle editor
- swarm status panel
- mode buttons
- real-time 2D top view for 6 aircraft
- click-to-set mission anchor behavior
- requested formation versus avoidance-adjusted formation overlay

Later, if needed, replace it with a standalone PyQt application.

## Current 2D avoidance path

The current first-pass avoidance design is intentionally 2D:

- operator publishes rectangular or cylindrical obstacles in the map plane
- `avoidance_2d` listens to `/swarm/formation_cmd` and `/swarm/obstacles`
- a safe anchor is published on `/swarm/formation_cmd_safe`
- `formation_controller` consumes the safe command topic
- the GUI renders dashed requested formation and solid safe formation in real time

This keeps the first implementation simple for fixed-wing swarm testing while still making avoidance behavior visible.

## Current repository notes

Your local repository is currently:

- `/home/chen/catkin_ws/PX4_Firmware`

The multi-vehicle helper script path in this repository is:

- `Tools/simulation/sitl_multiple_run.sh`
