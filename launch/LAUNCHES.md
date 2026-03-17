# Launch Guide

This folder contains the ROS launch entry points for the project.

Package name:
- `nexus_swarm_sim`

## Main Launch Files

### `base.launch`
- Starts the shared simulation base.
- Launches Gazebo with the selected world.
- Launches the `/uwb_simulator` node.
- Usually not called directly by hand.

### `uwb_only.launch`
- Lightweight smoke-test mode.
- Spawns simple dummy models.
- Does not require ArduPilot or MAVROS.
- Starts the 2D web dashboard on `http://localhost:8787` when `headless:=true` by default.
- Use this when you only want to verify UWB simulation and topics.

Example:
```bash
roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3 drone_prefix:=nexus
```

### `models_only.launch`
- Visual/physics simulation mode with real Gazebo drone models.
- Spawns Gazebo `iris_with_standoffs`-based models.
- Does not require ArduPilot.
- Starts the 2D web dashboard on `http://localhost:8787` when `headless:=true` by default.
- Use this when you want to see realistic models in Gazebo while keeping the setup simpler than full SITL.

Example:
```bash
roslaunch nexus_swarm_sim models_only.launch gui:=true headless:=false num_drones:=3 drone_prefix:=nexus
```

### `single_vehicle_sitl.launch`
- Single-vehicle ArduPilot integration mode.
- Uses the local `~/ardupilot_gazebo` world and model with `libArduPilotPlugin.so`.
- Starts `sim_vehicle.py` and MAVROS for one vehicle.
- Starts the 2D web dashboard on `http://localhost:8787` when `headless:=true` by default.
- This is the correct place to validate SITL integration before rebuilding any multi-vehicle workflow.

Example:
```bash
roslaunch nexus_swarm_sim single_vehicle_sitl.launch
```

### `full_swarm.launch`
- Full multi-vehicle ArduPilot mode.
- Starts Gazebo, multiple SITL instances, MAVROS, and the `/uwb_simulator`.
- Starts the 2D web dashboard on `http://localhost:8787` when `headless:=true` by default.
- This is the main end-to-end swarm launch.

Example:
```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus
```

## Spawn Launch Files

These are helper launch files. They are normally started by `scripts/swarm_launcher.py`, not manually.

### `spawn_dummy.launch`
- Spawns one dummy model.
- Used by `uwb_only.launch`.

### `spawn_model.launch`
- Spawns one Gazebo model from an SDF file.
- Used by `models_only.launch`.

### `spawn_sitl.launch`
- Starts one ArduPilot-backed drone instance with MAVROS.
- Used by `full_swarm.launch`.

## Practical Defaults

Dashboard defaults:
- `dashboard:=true` when `headless:=true`
- `dashboard:=false` when `headless:=false`
- `dashboard_host:=0.0.0.0`
- `dashboard_port:=8787`

If you do not override arguments:
- `vehicle_model:=iris`
- `drone_prefix:=nexus`
- `num_drones:=3`

This produces vehicles like:
- `nexus0`
- `nexus1`
- `nexus2`

## Which One Should I Use?

- If you only want to test UWB topics quickly: `uwb_only.launch`
- If you want the practical main mode today without SITL: `models_only.launch`
- If you want to validate ArduPilot integration the right way: `single_vehicle_sitl.launch`
- If you want the complete multi-vehicle stack: `full_swarm.launch`
