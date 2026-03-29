# relative_localization

## Purpose

`relative_localization` estimates relative pose or relative state between
vehicles using processed sensing outputs and other vehicle state sources.

## Inputs

- outputs from `swarm_sensing`
- MAVROS state topics
- optional GNSS / IMU / heading / altitude sources

## Responsibilities

- fuse processed UWB-derived measurements
- estimate relative vehicle state
- publish localization outputs usable by control and coordination layers

## Non-Goals

- raw UWB frame interpretation
- swarm decision-making
- simulation ownership

## Candidate Outputs

- `/swarm/relative_pose`
- `/swarm/relative_state`
- `/swarm/localization_quality`
