# Examples

This folder contains example nodes that consume the simulator's UWB topics.

The preferred low-level example path is:
- `raw_signal/raw_dstwr_exchange_consumer_example.py`
  Uses the current `RawUWBSignal` DS-TWR flow and exchange reconstruction.

Layout:
- `discovery/`
  Neighbor and topology-oriented consumers.
- `raw_signal/`
  Low-level `RawUWBSignal` and DS-TWR-oriented consumers.

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
- `discovery/neighbor_discovery_example.py` also makes sense in `uwb_only.launch`.
- `raw_signal/raw_dstwr_exchange_consumer_example.py` is useful when `raw_signal_protocol:=ds_twr`.
- `models_only.launch` is useful when you want real Gazebo visuals but not SITL.

## 1. Neighbor Discovery

Purpose:
- Discover other drones dynamically
- Track incoming UWB links to one drone

Run:
```bash
rosrun nexus_swarm_sim neighbor_discovery_example.py _drone_id:=nexus1 _drone_prefix:=nexus
```

Useful params:
- `_drone_id:=nexus1`
- `_drone_prefix:=nexus`
- `_topic_suffix:=range`
- `_max_range_m:=50.0`
- `_min_quality:=0.5`

Input topic:
- `/<drone_id>/uwb/range`

## 2. DS-TWR Exchange Consumer

Purpose:
- Subscribe to all `raw_signal` topics relevant to one drone
- Reconstruct DS-TWR exchanges by `exchange_seq`
- Detect complete and incomplete exchanges

Run:
```bash
rosrun nexus_swarm_sim raw_dstwr_exchange_consumer_example.py _drone_id:=nexus1 _drone_prefix:=nexus
```

Useful params:
- `_drone_id:=nexus1`
- `_drone_prefix:=nexus`
- `_exchange_timeout_s:=0.75`

Input topics:
- `/<drone>/uwb/raw_signal`

Notes:
- Complete exchanges require `POLL`, `RESP`, and `FINAL`.
- Missing `RESP` or `FINAL` frames are reported as incomplete exchanges after timeout.

## Typical Combined Flow

Terminal 1:
```bash
roslaunch nexus_swarm_sim full_swarm.launch num_drones:=3 vehicle_model:=iris drone_prefix:=nexus
```

Terminal 2:
```bash
rosrun nexus_swarm_sim raw_dstwr_exchange_consumer_example.py _drone_id:=nexus1 _drone_prefix:=nexus
```

Terminal 3:
```bash
rosrun nexus_swarm_sim neighbor_discovery_example.py _drone_id:=nexus1 _drone_prefix:=nexus
```
