# Nexus Swarm Sim Package Scope

## Purpose

`nexus_swarm_sim` is the simulation-environment package for the Nexus multi-UAV workflow.

Its job is to provide a realistic, repeatable ROS Noetic and Gazebo-based test environment for:

- multi-vehicle world bringup
- ArduPilot SITL integration
- MAVROS connectivity
- inter-vehicle UWB sensing simulation
- LOS/NLOS-aware ranging behavior and related measurement effects

The package exists to let downstream robotics software be developed and validated against a controlled simulated environment before real hardware is available or fully integrated.

## What This Package Owns

This package is responsible for environment and sensor-side simulation concerns.

That includes:

- Gazebo world loading and shared simulation infrastructure
- vehicle spawning and swarm bringup orchestration
- ArduPilot SITL launch flow for supported scenarios
- MAVROS bridge launch and namespace organization
- UWB-related measurement generation
- Gazebo-backed LOS raycast checks and channel-effect simulation
- simulation-time debug outputs and ground-truth-assisted validation
- operator-facing runtime utilities such as the built-in dashboard

In short, this package produces the simulated world and the simulated sensor outputs that other packages consume.

## What This Package Does Not Own

This package is not the home of higher-level robotics algorithms.

It should not become the primary implementation location for:

- TWR solving logic
- relative localization
- multi-agent state estimation
- swarm autonomy
- task allocation
- formation control
- mission planning
- flight-control policies beyond simulator bringup needs

Those belong in separate packages that consume the interfaces exposed by `nexus_swarm_sim`.

## Architectural Boundary

The intended architectural split is:

- `nexus_swarm_sim`: environment, simulated vehicles, simulated UWB measurements, runtime tooling
- downstream packages: measurement processing, estimation, localization, coordination, and autonomy

This boundary is intentional. It keeps the simulation package reusable while allowing higher-level algorithms to be transferred later to real hardware with minimal refactoring.

## Design Goal

The main design goal is not only to "run a simulation", but to expose a sensor-facing interface that is close enough to a real deployment workflow that downstream packages can be developed now and migrated later.

The package should therefore prioritize:

- stable ROS interfaces
- realistic timing and measurement behavior
- reproducible simulation scenarios
- clear separation between simulated sensor generation and algorithmic interpretation
- debug visibility without polluting the long-term production interface

## Expected Downstream Use

Other packages should be able to:

- subscribe to UWB measurement topics
- consume MAVROS and vehicle namespaces
- replay logged simulation data
- validate estimation and coordination logic
- swap the simulated measurement source for a real hardware source later without redesigning the full software stack

## Non-Goals

The following are explicitly out of scope for this package unless needed strictly for simulator validation:

- final localization pipelines
- production swarm decision-making
- hardware driver implementation for real UWB devices
- research-specific estimator tuning embedded into the simulator
- packaging the simulator as the final operational flight stack

## Success Criteria

`nexus_swarm_sim` is successful when it provides:

- a stable multi-UAV simulation environment
- credible UWB measurement behavior for algorithm development
- enough realism to de-risk later hardware integration
- enough modularity that higher-level packages remain portable

## Summary

`nexus_swarm_sim` is the simulation and sensing substrate of the system.

It should generate the world, the vehicles, and the measurement stream.
It should not own the algorithms that interpret those measurements into localization, coordination, or autonomy.
