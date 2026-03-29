# uwb_tools

## Purpose

`uwb_tools` collects developer-oriented support tooling around simulated and
later real UWB data.

## Inputs

- rosbag recordings
- simulator outputs
- ground-truth topics
- processed sensing outputs

## Responsibilities

- log inspection
- replay helpers
- offline plotting
- evaluation scripts
- simulated-vs-ground-truth comparison
- diagnostics and reporting

## Non-Goals

- core runtime sensing logic
- localization
- coordination

## Typical Use

- debug a strange ranging condition
- compare simulated and processed outputs
- evaluate timing or channel-model changes
- build repeatable offline test workflows
