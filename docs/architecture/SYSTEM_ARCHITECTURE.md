# System Architecture

## Purpose

This document describes the recommended high-level system architecture around
`nexus_swarm_sim` and the downstream packages that should consume its outputs.

The main architectural rule is:

- `nexus_swarm_sim` owns simulation, vehicle bringup, and UWB-facing data generation
- downstream packages own interpretation, estimation, coordination, and mission logic

This boundary keeps the simulator reusable and preserves a clean path toward
future sim-to-real migration.

## Package Roles

### `nexus_swarm_sim`

Owns:

- Gazebo world setup
- ArduPilot SITL integration
- MAVROS launch and namespace organization
- simulated `RawUWBSignal` publication
- simulated `UwbRange` publication
- LOS/NLOS and channel-effect simulation
- operator-facing dashboard and simulation-time tools

### `swarm_sensing`

Owns:

- low-level UWB link interpretation
- DS-TWR reconstruction
- link quality and exchange status interpretation
- payload decoding from UWB frames

### `swarm_shared_state`

Owns:

- normalization of payload-carried neighbor state
- freshness checks
- validity checks
- a stable downstream representation of neighbor-shared navigation data

### `relative_localization`

Owns:

- frame conversion and alignment
- relative pose and relative state estimation
- optional fusion of payload-carried GPS/state with UWB-derived ranging outputs
- quality scoring for relative estimates

### `swarm_coordination`

Owns:

- formation control
- collision avoidance
- safe target generation
- vehicle-level coordination outputs

### `swarm_mission`

Owns:

- mission phases
- QR- and task-driven mission logic
- high-level task objectives
- mission-to-coordination intent generation

### `uwb_tools`

Owns:

- logging
- replay
- offline evaluation
- plotting and diagnostics

## Data Flow

The intended data flow is:

1. `nexus_swarm_sim` publishes simulated UWB, vehicle, and debug data.
2. `swarm_sensing` interprets UWB link traffic and decodes frame payloads.
3. `swarm_shared_state` turns decoded payloads into stable neighbor state.
4. `relative_localization` estimates relative state from local state, neighbor state, and optional UWB-derived measurements.
5. `swarm_coordination` consumes relative state and generates safe formation-aware targets.
6. `swarm_mission` supplies high-level task objectives to the coordination layer.
7. `uwb_tools` supports logging, replay, and evaluation across the stack.

## Mermaid Overview

```mermaid
flowchart TD
    subgraph SIM["Simulation Substrate"]
        SIM1["nexus_swarm_sim"]
        SIM2["Gazebo world and vehicles"]
        SIM3["ArduPilot SITL bringup"]
        SIM4["MAVROS bridges"]
        SIM5["UWB simulator"]
        SIM6["Dashboard and monitor tools"]

        SIM1 --> SIM2
        SIM1 --> SIM3
        SIM1 --> SIM4
        SIM1 --> SIM5
        SIM1 --> SIM6
    end

    subgraph SENSE["Sensing Layer"]
        SEN1["swarm_sensing"]
        SEN2["DS-TWR reconstruction"]
        SEN3["Exchange status and link quality"]
        SEN4["Payload decode"]
        SEN5["Neighbor measurement outputs"]

        SEN1 --> SEN2
        SEN1 --> SEN3
        SEN1 --> SEN4
        SEN1 --> SEN5
    end

    subgraph STATE["Shared State Layer"]
        ST1["swarm_shared_state"]
        ST2["Neighbor state buffer"]
        ST3["Freshness and validity checks"]
        ST4["Normalized shared navigation state"]

        ST1 --> ST2
        ST1 --> ST3
        ST1 --> ST4
    end

    subgraph LOC["Localization Layer"]
        LOC1["relative_localization"]
        LOC2["Frame transform"]
        LOC3["Relative pose/state estimator"]
        LOC4["Optional GPS plus UWB fusion"]
        LOC5["Estimate quality output"]

        LOC1 --> LOC2
        LOC1 --> LOC3
        LOC1 --> LOC4
        LOC1 --> LOC5
    end

    subgraph COORD["Coordination Layer"]
        CO1["swarm_coordination"]
        CO2["Formation manager"]
        CO3["Collision avoidance"]
        CO4["Safe target generator"]
        CO5["Vehicle command adapter"]

        CO1 --> CO2
        CO1 --> CO3
        CO1 --> CO4
        CO1 --> CO5
    end

    subgraph MISSION["Mission Layer"]
        MI1["swarm_mission"]
        MI2["Mission phase manager"]
        MI3["QR and task logic"]
        MI4["Mission objective publisher"]

        MI1 --> MI2
        MI1 --> MI3
        MI1 --> MI4
    end

    subgraph TOOLS["Support and Evaluation"]
        T1["uwb_tools"]
        T2["Logging"]
        T3["Replay"]
        T4["Offline evaluation"]
        T5["Diagnostics and plotting"]

        T1 --> T2
        T1 --> T3
        T1 --> T4
        T1 --> T5
    end

    SIM5 -->|"raw_signal"| SEN1
    SIM5 -->|"simulated range"| LOC1
    SIM4 -->|"vehicle pose, velocity, gps"| LOC1
    SEN4 -->|"decoded payload"| ST1
    SEN5 -->|"neighbor measurements and link state"| LOC1
    ST4 -->|"neighbor shared state"| LOC1
    LOC5 -->|"relative pose, relative state, quality"| CO1
    MI4 -->|"mission objectives, formation mode, task targets"| CO1
    CO5 -->|"safe commands or targets"| SIM4

    SIM1 -->|"simulation logs and debug streams"| T1
    SEN1 -->|"inspection streams"| T1
    LOC1 -->|"estimation traces"| T1
    CO1 -->|"coordination diagnostics"| T1
```

## Responsibility Boundary

The intended responsibility split is:

- `nexus_swarm_sim`: produce simulated world state and sensor-facing outputs
- `swarm_sensing`: interpret low-level UWB traffic
- `swarm_shared_state`: normalize neighbor-broadcast state
- `relative_localization`: estimate relative geometry and motion
- `swarm_coordination`: decide safe coordinated motion
- `swarm_mission`: decide what the swarm should do
- `uwb_tools`: inspect and evaluate the stack

## Design Notes

- `RawUWBSignal` should not be consumed directly by localization, coordination, or mission packages.
- Payload decoding should happen before localization.
- Collision avoidance belongs to `swarm_coordination`, not to `relative_localization`.
- Mission logic should not bypass the coordination layer and command vehicles directly.
- `uwb_tools` should remain a support package rather than a runtime control dependency.
