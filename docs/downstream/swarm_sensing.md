# swarm_sensing

## Purpose

`swarm_sensing` is the first downstream package to build.

Its role is to consume low-level UWB-oriented outputs and convert them into
stable, usable sensing outputs for the rest of the swarm stack.

This package is the intended bridge between:

- simulated UWB sources today
- real UWB hardware sources later

## Inputs

- `/<vehicle>/uwb/raw_signal`
- optionally `/<vehicle>/uwb/range`
- vehicle namespaces and IDs
- later: real hardware low-level UWB frames or driver outputs

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

- `/<vehicle>/uwb/range_estimate`
- `/<vehicle>/uwb/link_status`
- `/swarm/neighbors`
- `/swarm/sensing_health`

## Why It Comes First

- it is the cleanest first boundary after the simulator
- it makes sim-to-real migration easier
- it prevents signal interpretation logic from leaking into `nexus_swarm_sim`
