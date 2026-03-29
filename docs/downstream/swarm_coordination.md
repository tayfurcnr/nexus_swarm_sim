# swarm_coordination

## Purpose

`swarm_coordination` owns higher-level multi-vehicle behavior.

It should sit above sensing and localization packages and consume their outputs
instead of consuming simulator internals directly.

## Inputs

- outputs from `relative_localization`
- operator goals
- mission plans
- safety and health status

## Responsibilities

- formation logic
- coordination behavior
- role allocation
- task-level decisions
- per-vehicle target generation

## Non-Goals

- low-level UWB interpretation
- final state estimation internals
- simulator implementation details

## Candidate Outputs

- `/<vehicle_ns>/swarm/coordination/targets`
- `/<vehicle_ns>/swarm/coordination/formation_cmd`
- per-vehicle setpoints or task commands

Example:

- `/nexus/1/swarm/coordination/targets`
