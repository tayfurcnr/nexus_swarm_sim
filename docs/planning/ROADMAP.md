# Nexus Swarm Sim Roadmap

## Objective

This roadmap defines the intended evolution of `nexus_swarm_sim` as a reusable
simulation substrate for multi-vehicle Gazebo and ArduPilot workflows with
simulated UWB outputs.

The package is not intended to become the home of downstream localization,
range-solving, coordination, or autonomy algorithms. Its role is to provide a
stable, realistic-enough environment and UWB-facing data stream that downstream
packages can consume now and later replace with real hardware sources with
limited architectural change.

## Working Assumptions

The roadmap below assumes:

- one UWB node per vehicle
- vehicle-level identifiers in published topics and messages
- `RawUWBSignal` is the preferred future-facing low-level interface
- `UwbRange` remains a processed simulator-side helper output

Explicit multi-node-per-vehicle topology is not a current roadmap target.

## Guiding Principles

- Keep environment generation and algorithm interpretation separate.
- Prefer stable ROS contracts over simulator-specific shortcuts.
- Improve realism where it changes downstream software behavior.
- Preserve a usable low-level UWB-facing interface for future sim-to-real work.
- Keep simulation-only diagnostics available without making downstream packages depend on them.
- Make debugging, replay, and reproducibility practical.

## Current State

The following are already present in the repository:

- a documented UWB-facing interface contract
- `UwbRange` and `RawUWBSignal` message definitions
- per-vehicle UWB topic naming
- single-node-per-vehicle mounting assumption
- Gazebo LOS raycast support
- configurable timing, dropout, jitter, outlier, and NLOS-related effects
- DS-TWR-oriented low-level raw signal publishing
- monitor, dashboard, and example consumer tooling

Relevant references:

- [UWB Interface Contract](/home/kairos/swarm_ws/src/nexus_swarm_sim/docs/contracts/UWB_INTERFACE_CONTRACT.md)
- [UWB Simulator](/home/kairos/swarm_ws/src/nexus_swarm_sim/src/uwb_simulator.cpp)
- [LOS Raycast Plugin](/home/kairos/swarm_ws/src/nexus_swarm_sim/src/los_raycast_plugin.cpp)

## Remaining Roadmap

## Phase 1: Contract Maintenance

### Goal

Keep the published UWB contract stable and explicit as the simulator evolves.

### Work Items

- Keep `RawUWBSignal` aligned with the intended future hardware-facing interface.
- Keep `UwbRange` clearly documented as a processed simulator helper output.
- Tighten semantics for:
  - timestamps
  - validity/status behavior
  - quality fields
  - DS-TWR exchange grouping and failure behavior
- Document any contract changes before they are introduced.

### Exit Criteria

- Downstream packages can treat the measurement interface as stable.
- Contract changes are deliberate, documented, and minimal.

## Phase 2: Timing, Scheduling, and Reliability Realism

### Goal

Improve low-level behavioral realism in ways that materially affect downstream
range extraction, synchronization, and estimation logic.

### Work Items

- Strengthen the current timing model with:
  - configurable jitter
  - stale measurement behavior
  - variable publication cadence
  - per-link reliability effects
  - burst-loss style behavior where useful
- Ensure dropped, delayed, and degraded measurements are explicit in published outputs.
- Improve status/validity semantics for downstream consumers.
- Add diagnostic metadata where useful for correlation and replay.

### Exit Criteria

- Timing behavior is configurable and testable.
- Failure and degradation modes are explicit rather than hidden.
- Downstream packages can be stress-tested before hardware exists.

## Phase 3: Channel and Propagation Realism

### Goal

Move beyond a basic LOS/NLOS split toward more credible per-link behavior.

### Work Items

- Keep Gazebo raycast as the geometric visibility foundation.
- Improve channel behavior with effects such as:
  - obstruction severity
  - distance-sensitive reliability
  - persistent per-link bias
  - body-shadowing-style degradation where justified
  - angle-dependent effects if they materially help downstream testing
- Refine quality-related outputs so they reflect simulated channel state more credibly.
- Keep all such effects externally configurable and reproducible.

### Exit Criteria

- The simulator can represent materially different link conditions beyond a single binary LOS/NLOS result.
- Channel outputs are useful for downstream robustness testing.
- Known-good and known-bad scenarios can be reproduced from configuration.

## Phase 4: Debuggability, Logging, and Replay

### Goal

Make UWB streams inspectable, recordable, and reusable outside live Gazebo sessions.

### Work Items

- Define a clear logging workflow for UWB outputs.
- Define a replay workflow for downstream package development.
- Add tools to compare simulated measurements against ground truth or simulator diagnostics.
- Separate geometry effects, timing effects, and injected channel/noise effects in diagnostics where practical.

### Exit Criteria

- Downstream packages can be tested from recorded simulation output.
- Simulator behavior can be analyzed offline.
- Realism can be evaluated quantitatively, not only visually.

## Phase 5: Sim-to-Real Readiness

### Goal

Keep this package useful as a precursor to real UWB-backed systems instead of a
Gazebo-only dead end.

### Work Items

- Keep sensor-facing outputs aligned with the intended real-hardware integration path.
- Avoid Gazebo-only assumptions in the consumer-facing contract where possible.
- Keep real-hardware-only implementation concerns out of this package.
- Validate that downstream packages can swap simulated and real sources with minimal internal redesign.

### Exit Criteria

- The package exposes interfaces that are practical to mirror from real hardware.
- The simulation package remains useful after a real UWB stack exists.
- Downstream packages can keep most of their internal logic unchanged when moving off simulation.

## Near-Term Priorities

The highest-priority next steps are:

1. Strengthen timing and reliability modeling.
2. Improve channel and propagation realism.
3. Add replay-oriented logging and evaluation support.
4. Keep the low-level UWB-facing contract stable.

## Out of Scope

The following are intentionally outside this package except where simulator
validation strictly requires them:

- final TWR solvers
- localization pipelines
- swarm state estimation
- coordination and autonomy logic
- production UWB hardware drivers
- multi-node-per-vehicle UWB topology

Those belong in downstream packages that consume the interfaces provided here.

## Success Condition

`nexus_swarm_sim` should provide:

- a stable multi-UAV simulation environment
- credible UWB behavior for downstream development
- a low-level UWB-facing interface suitable for downstream signal-processing and range-extraction packages
- debugging and replay support that makes realism inspectable
- clear boundaries that support later migration to real hardware

When that condition is met, downstream packages should be able to develop
against this simulator now and transition later to real hardware with limited
architectural change.
