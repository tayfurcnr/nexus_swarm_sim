# swarm_sensing

## Purpose

`swarm_sensing` is the first downstream package to build.

Its role is to consume low-level UWB-oriented outputs and convert them into
stable, usable sensing outputs for the rest of the swarm stack.

This package is the intended bridge between:

- simulated UWB sources today
- real UWB hardware sources later

## Inputs

Placeholder note:

- `/<vehicle_ns>/...` means "the ROS namespace of one vehicle"
- current canonical examples are `/nexus/1/...`, `/nexus/2/...`
- this document uses `vehicle_ns` as a reusable template, not as a second naming scheme

- `/<vehicle_ns>/uwb/raw_signal`
- vehicle namespaces and IDs such as `nexus/1`, `nexus/2`
- later: real hardware low-level UWB frames or driver outputs

Current simulator example inputs:

- `/nexus/1/uwb/raw_signal`
- `/nexus/2/uwb/raw_signal`

## Responsibilities

- reconstruct DS-TWR exchanges
- detect incomplete or degraded exchanges
- classify link health
- infer whether a measurement should be trusted
- derive processed sensing outputs for upper layers
- provide a stable package boundary between low-level UWB traffic and higher-level estimation

## Non-Goals

- final localization
- swarm coordination
- mission logic
- direct simulator ownership

## Candidate Outputs

Canonical topic style:

- template form: `/<vehicle_ns>/swarm/sensing/...`
- current simulator examples: `/nexus/1/swarm/sensing/...`, `/nexus/2/swarm/sensing/...`

- `/<vehicle_ns>/swarm/sensing/neighbor_measurements`
- `/<vehicle_ns>/swarm/sensing/link_state`
- `/<vehicle_ns>/swarm/sensing/exchange_status`
- `/<vehicle_ns>/swarm/sensing/diagnostics`
- `/<vehicle_ns>/swarm/sensing/stats`

Recommended topic shape:

- all outputs stay under the vehicle namespace
- example vehicle-scoped topic: `/nexus/1/swarm/sensing/link_state`

## Why It Comes First

- it is the cleanest first boundary after the simulator
- it makes sim-to-real migration easier
- it prevents signal interpretation logic from leaking into `nexus_swarm_sim`
