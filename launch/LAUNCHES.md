# Launch Reference

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

![Nexus Swarm Sim Logo](../docs/logo.png)

This document describes the public ROS launch entry points shipped with `nexus_swarm_sim` and the role of each one.

## Table Of Contents

- [Launch Organization](#launch-organization)
- [Public Launches](#public-launches)
- [Helper Launches](#helper-launches)
- [Default Behavior](#default-behavior)
- [Recommended Usage Sequence](#recommended-usage-sequence)
- [Selection Guidance](#selection-guidance)

## Launch Organization

Public entry points:
- `base.launch`
- `uwb_only.launch`
- `models_only.launch`
- `single_vehicle_sitl.launch`
- `full_swarm.launch`

Helper launches:
- `spawn_dummy.launch`
- `spawn_model.launch`
- `spawn_sitl.launch`

Helper launch files are typically started by [swarm_launcher.py](../scripts/swarm_launcher.py), not by hand.

## Public Launches

### `base.launch`

Purpose:
- starts the shared simulation infrastructure
- launches Gazebo with the selected world
- starts the UWB simulator
- optionally starts the dashboard

Use this when:
- you are composing a custom launch flow
- you want the simulation base without directly spawning vehicles

### `uwb_only.launch`

Purpose:
- lightweight smoke-test mode
- spawns simple dummy models from this repository
- does not require ArduPilot or MAVROS

Use this when:
- you want to validate Gazebo and the UWB stack first
- you want the fastest environment smoke test

Example:

```bash
roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3 drone_prefix:=nexus
```

### `models_only.launch`

Purpose:
- visual and physics simulation mode with Gazebo vehicle models
- no ArduPilot SITL
- no MAVROS dependency

Use this when:
- you want realistic Gazebo visuals
- you want multi-model testing without the full SITL chain

Example:

```bash
roslaunch nexus_swarm_sim models_only.launch gui:=true headless:=false num_drones:=3 drone_prefix:=nexus
```

### `single_vehicle_sitl.launch`

Purpose:
- single-vehicle ArduPilot validation mode
- launches one Gazebo vehicle, one SITL instance, and one MAVROS bridge

Use this when:
- you want to validate `~/ardupilot` and `~/ardupilot_gazebo`
- you want to verify the full stack on one vehicle before multi-vehicle bringup

Example:

```bash
roslaunch nexus_swarm_sim single_vehicle_sitl.launch
```

### `full_swarm.launch`

Purpose:
- full multi-vehicle ArduPilot mode
- launches Gazebo, multiple SITL instances, MAVROS bridges, and the UWB simulator

Use this when:
- ArduPilot and `ardupilot_gazebo` are already installed
- single-vehicle SITL bringup is already working
- you want the main end-to-end swarm workflow

Example:

```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus
```

## Helper Launches

### `spawn_dummy.launch`
- spawns one dummy vehicle model
- used internally by `uwb_only.launch`

### `spawn_model.launch`
- spawns one Gazebo model from an SDF file
- used internally by `models_only.launch`

### `spawn_sitl.launch`
- starts one ArduPilot-backed vehicle instance
- used internally by `full_swarm.launch` and `single_vehicle_sitl.launch`

## Default Behavior

Common defaults:
- `vehicle_model:=iris`
- `drone_prefix:=nexus`
- `num_drones:=3` for lightweight modes
- `num_drones:=12` in `full_swarm.launch`

Dashboard defaults:
- `full_swarm.launch`: `dashboard:=true`
- `models_only.launch`: `dashboard:=$(arg headless)`
- `single_vehicle_sitl.launch`: `dashboard:=$(arg headless)`
- `uwb_only.launch`: `dashboard:=$(arg headless)`
- `dashboard_host:=0.0.0.0`
- `dashboard_port:=8787`

Generated vehicle names follow the pattern:
- `nexus1`
- `nexus2`
- `nexus3`
- ...

## Recommended Usage Sequence

For a new machine or a fresh setup:
1. `uwb_only.launch`
2. `models_only.launch`
3. `single_vehicle_sitl.launch`
4. `full_swarm.launch`

## Selection Guidance

Use `uwb_only.launch` for the fastest smoke test.

Use `models_only.launch` when you want Gazebo visuals without ArduPilot.

Use `single_vehicle_sitl.launch` when you want to validate the full integration path safely on one vehicle.

Use `full_swarm.launch` when the full dependency stack is already installed and validated.
