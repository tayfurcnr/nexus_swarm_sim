# Documentation Map

This directory contains the package-level documents that define scope, contracts, planning, and hardware references.

## Architecture

- [architecture/PACKAGE_SCOPE.md](architecture/PACKAGE_SCOPE.md): package ownership, boundaries, non-goals, and intended downstream split
- [architecture/SYSTEM_ARCHITECTURE.md](architecture/SYSTEM_ARCHITECTURE.md): high-level system architecture and package-to-package data flow
- [architecture/DOWNSTREAM_PACKAGE_PLAN.md](architecture/DOWNSTREAM_PACKAGE_PLAN.md): mermaid overview of the planned simulator-to-downstream package split

## Downstream Packages

- [downstream/README.md](downstream/README.md): overview of planned external packages and recommended implementation order
- [downstream/swarm_sensing.md](downstream/swarm_sensing.md): first downstream package, focused on interpreting low-level UWB traffic
- [downstream/relative_localization.md](downstream/relative_localization.md): planned relative state estimation package
- [downstream/swarm_coordination.md](downstream/swarm_coordination.md): planned swarm behavior and coordination package
- [downstream/uwb_tools.md](downstream/uwb_tools.md): planned replay, evaluation, and diagnostics package

## Contracts

- [contracts/UWB_INTERFACE_CONTRACT.md](contracts/UWB_INTERFACE_CONTRACT.md): UWB topic, message, timestamp, and consumer-facing interface semantics

## Planning

- [planning/ROADMAP.md](planning/ROADMAP.md): phased roadmap for realism, contracts, replay, and sim-to-real readiness

## Hardware References

- [hardware/uwb/qorvo/datasheets/README.md](hardware/uwb/qorvo/datasheets/README.md): current Qorvo DW3000/DWM3000 reference set used to shape the low-level UWB interface
