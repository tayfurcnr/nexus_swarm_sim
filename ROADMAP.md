# Nexus Swarm Sim Roadmap

## Objective

This roadmap defines the intended evolution of `nexus_swarm_sim` as a simulation-environment and sensor-simulation package.

The package is not intended to become the home of downstream localization, TWR-solving, or swarm-autonomy algorithms. Its role is to provide a realistic and reusable simulation substrate that other packages can consume with minimal rework when transitioning to real hardware.

The roadmap therefore prioritizes:

- interface stability
- simulation realism where it materially affects downstream systems
- clean boundaries between simulation and algorithm packages
- reproducibility and debugging support
- migration readiness for future real UWB hardware integration

## Guiding Principles

- Keep environment generation and algorithm interpretation separate.
- Prefer stable ROS contracts over simulator-specific shortcuts.
- Improve realism first where it changes downstream software behavior.
- Isolate simulation-only debug data from long-term sensor-facing interfaces.
- Build toward a workflow where downstream packages can swap simulated and real measurement sources with limited changes.

## Phase 1: Interface and Contract Stabilization

### Goal

Define and stabilize the sensor-facing outputs that downstream packages will rely on.

### Work Items

- Review the current `UwbRange` and `RawUWBSignal` messages against long-term sim-to-real needs.
- Decide whether the package should continue exposing processed range outputs, lower-level measurement outputs, or both.
- Define a clear contract for:
  - topic names
  - namespaces
  - frame usage
  - source and destination identifiers
  - timestamp semantics
  - quality and validity semantics
- Separate simulation-only fields from production-like fields where needed.
- Document the chosen interface so downstream packages can treat it as stable.

### Exit Criteria

- A clear measurement contract exists and is documented.
- Message semantics are stable enough for downstream package development.
- Simulation-specific debug fields are explicitly identified.

## Phase 2: UWB Topology and Sensor Modeling

### Goal

Make the simulated UWB setup structurally closer to real deployments.

### Work Items

- Add explicit support for per-vehicle UWB node topology.
- Allow configuration of:
  - node count per vehicle
  - node identifiers
  - body-frame mounting offsets
  - orientation metadata where relevant
- Ensure measurement outputs are traceable to specific nodes, not only vehicle names.
- Extend configuration so future real-hardware layouts can be mirrored in simulation.

### Exit Criteria

- The simulator can represent more than a single implicit UWB node per vehicle.
- Mounting geometry is configurable and visible in the measurement interface.
- Downstream packages can reason about node-level rather than only vehicle-level measurements.

## Phase 3: Timing, Scheduling, and Transport Realism

### Goal

Improve behavioral realism in the parts of the measurement pipeline that affect downstream ranging and estimation logic.

### Work Items

- Strengthen the existing timing model with:
  - configurable jitter
  - burst packet loss
  - stale measurement behavior
  - variable publication cadence
  - per-link reliability effects
- Add clearer status and validity indicators for downstream consumers.
- Introduce sequence and diagnostic metadata where useful for correlation and replay.
- Ensure dropped, delayed, and degraded measurements are observable rather than silently hidden.

### Exit Criteria

- Timing behavior is configurable and testable.
- Measurement validity and failure modes are explicit in the published data.
- Downstream packages can be validated against degraded-link conditions before hardware is available.

## Phase 4: Channel and Propagation Realism

### Goal

Move from a binary LOS/NLOS model toward a more credible link-behavior model.

### Work Items

- Keep Gazebo LOS raycast support as the geometric visibility foundation.
- Add richer channel-behavior modeling, such as:
  - obstruction severity
  - distance-sensitive reliability
  - body shadowing effects
  - angle-dependent degradation where justified
  - persistent per-link bias behavior
- Refine quality-related outputs so they better reflect channel state rather than only synthetic placeholders.
- Preserve reproducibility by keeping channel parameters externally configurable.

### Exit Criteria

- The simulator can represent materially different link conditions beyond a single LOS/NLOS decision.
- Channel-related outputs are useful for downstream robustness testing.
- Scenario configuration can reproduce known-good and known-bad link conditions.

## Phase 5: Debuggability, Logging, and Replay

### Goal

Make simulated measurement streams inspectable, recordable, and reusable outside live Gazebo sessions.

### Work Items

- Define a clear logging and replay workflow for UWB outputs.
- Support replay-driven development for downstream packages.
- Add evaluation tooling to compare simulated measurements against available ground truth.
- Expose simulator diagnostics that help distinguish geometry effects, timing effects, and injected noise effects.

### Exit Criteria

- Downstream packages can be tested from recorded simulation data.
- Simulator behavior can be analyzed offline.
- Measurement realism can be evaluated quantitatively rather than only visually.

## Phase 6: Sim-to-Real Readiness

### Goal

Ensure this package remains a useful precursor to later real-hardware integration rather than a simulator-only dead end.

### Work Items

- Align sensor-facing outputs with the intended real-hardware integration path.
- Minimize assumptions that only hold in Gazebo.
- Keep real-hardware-only concerns out of this package while preserving compatibility with future drivers.
- Validate that downstream packages can replace the simulated source with a real source without major architectural changes.

### Exit Criteria

- The package exposes interfaces that are practical to mirror from real hardware.
- The simulation package remains reusable after the real UWB stack is introduced.
- Downstream localization and coordination packages can keep most of their internal logic unchanged when moving off simulation.

## Near-Term Priorities

The highest-priority next steps are:

1. Stabilize the UWB measurement contract.
2. Clarify which outputs are production-like and which are simulation-only.
3. Add explicit node-level topology support.
4. Strengthen timing and reliability modeling.
5. Add replay-oriented logging and evaluation support.

## Out of Scope

The following are intentionally not roadmap targets for this package except where simulator validation strictly requires them:

- final TWR solvers
- localization pipelines
- swarm state estimation
- coordination and autonomy logic
- production UWB hardware drivers

Those belong in downstream packages that consume the interfaces provided here.

## Success Condition

`nexus_swarm_sim` should evolve into a package that provides:

- a stable multi-UAV simulation environment
- credible UWB sensor behavior for downstream development
- clear interfaces that support later hardware migration
- enough realism to make early algorithm work meaningful
- enough separation of concerns to prevent simulator lock-in

When that condition is met, downstream packages should be able to develop against this simulator today and transition later to real hardware with limited architectural change.
