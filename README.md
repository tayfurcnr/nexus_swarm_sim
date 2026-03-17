# Swarm Gazebo Simulation

## Overview
This package now serves two roles:
- multi-vehicle Gazebo + ArduPilot + MAVROS bringup
- optional UWB ranging simulation on top of that swarm

The UWB layer provides realistic, dynamic DW3000-like range measurements. Drones are discovered at runtime from `/gazebo/model_states`, and links are created automatically as vehicles appear.

The package name is `swarm_gazebo_sim`.

## Dynamic Discovery
The system is built for scalability. You don't need to configure model names or static ranging pairs.
*   **Automatic Detection**: Simulator listens to `/gazebo/model_states` and detects any model starting with a configurable prefix (default: `nexus`).
*   **Runtime Link Creation**: As soon as a new drone is detected, the simulator creates UWB publishers for it and initiates ranging links with all existing drones in the swarm.

## Features & Behavior
The simulation publishes per-drone UWB data on `/<drone_id>/uwb/range`. 
*   **Dropout**: Probability increases with distance (approx. 40% at 100m).
*   **Latency**: Measurements have ~5ms delay with jitter to mimic real hardware processing.
*   **Outliers**: 1-3% of measurements have random bias (simulating multipath).
*   **LOS/NLOS Detection (Probabilistic)**:
    *   **LOS (Line-of-Sight)**: Normal noise baseline.
    *   **NLOS (Non-Line-of-Sight)**: +15cm bias, increased uncertainty. P(NLOS) grows with distance and altitude difference.
*   **Update Rate**: Each drone pair is throttled independently to `update_rate_hz` (default: 15Hz).

## Topic Architecture

Each discovered drone publishes its own data:

| Topic | Message Type | Description |
|---|---|---|
| `/<drone_id>/uwb/range` | `UwbRange` | Processed ranges (distances, LOS, etc.) |
| `/<drone_id>/uwb/raw_signal` | `RawUWBSignal` | Raw ToA/SNR/RSSI from the drone |

MAVROS stays namespaced per vehicle, for example:
- `/<drone_id>/mavros/state`
- `/<drone_id>/mavros/local_position/pose`

## Run Modes

Primary integration mode:
- `full_swarm.launch` is the main end-to-end mode.
- It starts Gazebo, ArduPilot SITL, MAVROS, and the UWB simulator for multiple vehicles.
- Use this for real swarm testing.

Debug-only fallback:
- `uwb_only.launch` is only for smoke testing the UWB simulator without ArduPilot.
- It does not validate the full flight stack or MAVROS integration.

Recommended non-SITL mode:
- `models_only.launch` gives you real Gazebo drone models and working UWB simulation without ArduPilot.
- Use this when you want visual simulation but do not need the flight stack.

Incremental ArduPilot validation mode:
- `single_vehicle_sitl.launch` is the one-vehicle debugging entry point.
- It uses the local `~/ardupilot_gazebo` world/model/plugin setup and starts `sim_vehicle.py` plus MAVROS for one vehicle.
- Use this when you want to isolate SITL issues before scaling to a swarm.

## Quick Start

### 1. Build
```bash
catkin_make
source devel/setup.bash
```

### 2. Launch
Main full-stack launch:
```bash
roslaunch swarm_gazebo_sim full_swarm.launch gui:=true headless:=false num_drones:=3 drone_prefix:=nexus
```

Headless test run:
```bash
roslaunch swarm_gazebo_sim full_swarm.launch gui:=false headless:=true num_drones:=2 drone_prefix:=nexus
```

Non-SITL Gazebo-only mode:
```bash
roslaunch swarm_gazebo_sim models_only.launch gui:=true headless:=false num_drones:=3 drone_prefix:=nexus
```

UWB-only smoke test:
```bash
roslaunch swarm_gazebo_sim uwb_only.launch num_drones:=3 drone_prefix:=nexus
```

Single-vehicle SITL validation:
```bash
roslaunch swarm_gazebo_sim single_vehicle_sitl.launch
```

### 3. Monitor
The included monitor tool lists detected drones and their current links:
```bash
rosrun swarm_gazebo_sim swarm_uwb_monitor.py
```

Useful quick checks:
```bash
rostopic echo /nexus0/mavros/state
rostopic echo /nexus0/uwb/range
rostopic list | grep /uwb/
```

## Developing: Adding a New Node
To use this data in your cooperative localization algorithm, subscribe to the `/<drone_id>/uwb/range` topics.

**Python Example:**
```python
import rospy
from swarm_gazebo_sim.msg import UwbRange

def range_callback(msg):
    # msg.src_id = source drone ID (e.g., "drone0")
    # msg.dst_id = destination drone ID (e.g., "drone1")
    # msg.distance_3d = measured distance with realistic noise
    # msg.los = Line-of-Sight flag
    rospy.loginfo(f"Link: {msg.src_id} -> {msg.dst_id}: {msg.distance_3d:.3f}m")

rospy.init_node('my_localization_node')
# You can subscribe to all drones dynamically or specific ones
rospy.Subscriber('/<drone_id>/uwb/range', UwbRange, range_callback)
rospy.spin()
```

## Configuration
Tune simulation parameters (prefix, dropout, noise, etc.) in [config/uwb_simulator.yaml](/home/tayfurcnr/Desktop/Projects/swarm_uwb_sim/config/uwb_simulator.yaml).
Key parameters:
- `drone_prefix`
- `model_prefix`
- `update_rate_hz`
- `pub_topic_prefix`

For scenario launches, the generic knobs are:
- `vehicle_model`: underlying Gazebo/SITL vehicle model, currently `iris`
- `drone_prefix`: spawned namespace/model prefix such as `nexus`, `drone`, `quad`
- `num_drones`: number of vehicles to spawn
- `gui` / `headless`: Gazebo visualization mode
