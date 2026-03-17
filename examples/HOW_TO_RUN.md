# Examples: How To Run

This folder contains example nodes that consume the simulator's UWB topics.

## Before You Start

Start one of the simulator modes first.

Full integration mode:
```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus
```

Debug-only UWB mode:
```bash
roslaunch nexus_swarm_sim uwb_only.launch num_drones:=3 drone_prefix:=nexus
```

Gazebo model mode without ArduPilot:
```bash
roslaunch nexus_swarm_sim models_only.launch gui:=true headless:=false num_drones:=3 drone_prefix:=nexus
```

Notes:
- `payload_injector_example.py` is meant for the full ArduPilot/MAVROS path.
- `neighbor_discovery_example.py` and `signal_processor_example.py` also make sense in `uwb_only.launch`.
- `models_only.launch` is useful when you want real Gazebo visuals but not SITL.
- `payload_processor.py` expects messages from `range_with_payload`, so it is most useful together with `payload_injector_example.py`.

## 1. Neighbor Discovery

Purpose:
- Discover other drones dynamically
- Track incoming UWB links to one drone

Run:
```bash
rosrun nexus_swarm_sim neighbor_discovery_example.py _drone_id:=nexus0 _drone_prefix:=nexus
```

Useful params:
- `_drone_id:=nexus0`
- `_drone_prefix:=nexus`
- `_topic_suffix:=range`
- `_max_range_m:=50.0`
- `_min_quality:=0.5`

Input topic:
- `/<drone_id>/uwb/range`

## 2. Raw Signal Processor

Purpose:
- Subscribe to raw UWB signals
- Convert ToA to distance
- Apply simple filtering and publish processed outputs

Run:
```bash
rosrun nexus_swarm_sim signal_processor_example.py _drone_id:=nexus0 _drone_prefix:=nexus
```

Published topics:
- `/<drone_id>/uwb/distance`
- `/<drone_id>/uwb/processed_measurement`

Useful params:
- `_drone_id:=nexus0`
- `_drone_prefix:=nexus`
- `_enable_filtering:=true`
- `_filter_window:=5`

Input topic:
- `/<drone_id>/uwb/raw_signal`

## 3. Payload Injector

Purpose:
- Read one drone's pose
- Encode state into the `payload` field
- Republish outgoing UWB range messages to `range_with_payload`

Run:
```bash
rosrun nexus_swarm_sim payload_injector_example.py _drone_id:=nexus0
```

Published topic:
- `/<drone_id>/uwb/range_with_payload`

Important:
- This example subscribes to `/<drone_id>/mavros/local_position/pose`
- Use it with the full ArduPilot/MAVROS launch, not the UWB-only mode

## 4. Payload Processor

Purpose:
- Listen to enriched UWB messages
- Decode payload data
- Print embedded drone state

Run:
```bash
rosrun nexus_swarm_sim payload_processor.py _drone_prefix:=nexus
```

Default input topic pattern:
- `/<drone_id>/uwb/range_with_payload`

Useful params:
- `_drone_prefix:=nexus`
- `_topic_suffix:=range_with_payload`

## Typical Combined Flow

Terminal 1:
```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus
```

Terminal 2:
```bash
rosrun nexus_swarm_sim payload_injector_example.py _drone_id:=nexus0
```

Terminal 3:
```bash
rosrun nexus_swarm_sim payload_processor.py _drone_prefix:=nexus
```

Terminal 4:
```bash
rosrun nexus_swarm_sim neighbor_discovery_example.py _drone_id:=nexus1 _drone_prefix:=nexus _topic_suffix:=range_with_payload
```
