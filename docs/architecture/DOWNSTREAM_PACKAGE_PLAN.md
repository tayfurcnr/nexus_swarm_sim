# Downstream Package Plan

## Purpose

This document defines the recommended package split around `nexus_swarm_sim`.

The main architectural rule is:

- `nexus_swarm_sim` generates the simulated world and UWB-facing data
- downstream packages interpret, estimate, and decide

This separation keeps the simulator reusable and reduces refactoring later when
simulated sources are replaced with real hardware sources.

## Core Package Boundary

- `nexus_swarm_sim`
  - owns Gazebo world setup
  - owns vehicle spawning and SITL bringup
  - owns simulated UWB publication
  - owns debug, monitoring, and simulation-time tooling
- downstream packages
  - own signal interpretation
  - own range extraction
  - own localization
  - own swarm coordination and autonomy

## Recommended Package Split

### `nexus_swarm_sim`

Responsibilities:

- Gazebo and world setup
- ArduPilot SITL integration
- MAVROS launch integration
- simulated `UwbRange` publication
- simulated `RawUWBSignal` publication
- LOS/NLOS and channel-effect simulation
- runtime dashboard and monitor tools

### `nexus_uwb_processing`

Responsibilities:

- subscribe to `/<vehicle>/uwb/raw_signal`
- reconstruct DS-TWR exchanges
- detect incomplete or degraded exchanges
- derive range-oriented outputs from low-level UWB signal messages
- publish processed link results for downstream estimation

Suggested outputs:

- `/<vehicle>/uwb/range_estimate`
- `/<vehicle>/uwb/link_status`

### `nexus_relative_localization`

Responsibilities:

- consume processed UWB outputs
- consume vehicle state from MAVROS and related topics
- estimate relative positions or relative state between vehicles
- publish relative pose/state outputs for coordination layers

Suggested outputs:

- `/swarm/relative_pose`
- `/swarm/relative_state`

### `nexus_swarm_coordination`

Responsibilities:

- consume localization outputs
- consume mission or operator goals
- run formation, coordination, and task-level decision logic
- publish vehicle-level commands or targets

Suggested outputs:

- `/swarm/targets`
- `/swarm/formation_cmd`
- per-vehicle command topics

### `nexus_uwb_tools`

Responsibilities:

- logging helpers
- replay helpers
- offline evaluation scripts
- simulated-vs-ground-truth comparison tooling
- plotting and diagnostics

## Mermaid Overview

```mermaid
flowchart LR
    A[nexus_swarm_sim\nWorld + Vehicles + UWB Simulation] --> B[nexus_uwb_processing\nDS-TWR + Range Extraction]
    A --> E[nexus_uwb_tools\nLogging + Replay + Evaluation]
    B --> C[nexus_relative_localization\nRelative Pose / State Estimation]
    C --> D[nexus_swarm_coordination\nFormation + Task + Decision Logic]

    A --> A1["/<vehicle>/uwb/raw_signal"]
    A --> A2["/<vehicle>/uwb/range"]
    B --> B1["/<vehicle>/uwb/range_estimate"]
    B --> B2["/<vehicle>/uwb/link_status"]
    C --> C1["/swarm/relative_pose"]
    C --> C2["/swarm/relative_state"]
    D --> D1["/swarm/targets"]
    D --> D2["/swarm/formation_cmd"]
```

## Data Flow Summary

1. `nexus_swarm_sim` publishes simulated UWB and vehicle-context data.
2. `nexus_uwb_processing` turns low-level UWB traffic into usable ranging outputs.
3. `nexus_relative_localization` turns those outputs into relative state estimates.
4. `nexus_swarm_coordination` consumes those estimates to make swarm-level decisions.
5. `nexus_uwb_tools` supports replay, inspection, and offline evaluation across the stack.

## Near-Term Recommendation

If only one downstream package is created first, it should be:

- `nexus_uwb_processing`

Reason:

- it is the most natural first boundary after the simulator
- it allows `RawUWBSignal` consumers to evolve outside the simulation package
- it preserves a clean path toward later real-hardware-backed ranging inputs
