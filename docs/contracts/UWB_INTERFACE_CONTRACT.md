# UWB Interface Contract

## Purpose

This document defines the intended sensor-facing contract exposed by `nexus_swarm_sim` for simulated UWB data.

The goal is to make downstream package development stable and migration-friendly:

- downstream packages should be able to consume simulated UWB data now
- the same downstream packages should later be able to consume real UWB data with limited architectural change
- simulation-only diagnostics must remain available without being confused with production-like sensor outputs

This contract does not define localization, TWR solving, coordination, or autonomy logic.
It only defines what this package publishes and how downstream packages should interpret it.

## Scope

This contract covers:

- `nexus_swarm_sim/UwbRange`
- `nexus_swarm_sim/RawUWBSignal`
- per-vehicle UWB topic naming
- the current single-node-per-vehicle assumption
- semantic meaning of identifiers, timestamps, and fields
- separation between production-like fields and simulation-only fields

This contract does not cover:

- estimator internals
- decision-making logic
- real hardware drivers
- final real-hardware message definitions outside this package

## Topic Contract

Placeholder note:

- `/<vehicle_ns>/...` means "the ROS namespace of one vehicle"
- current canonical examples are `/nexus/1/...`, `/nexus/2/...`
- this document uses `vehicle_ns` as a reusable template, not as an alternative ID format

### Published consumer topics

Processed simulated range outputs are published per vehicle:

- `/<vehicle_ns>/uwb/range`

Current simulator examples:

- `/nexus/1/uwb/range`
- `/nexus/2/uwb/range`

Low-level signal-oriented outputs are also published per vehicle:

- `/<vehicle_ns>/uwb/raw_signal`

Current simulator examples:

- `/nexus/1/uwb/raw_signal`
- `/nexus/2/uwb/raw_signal`

When `raw_signal_protocol` is set to `ds_twr`, this topic carries a simulated
double-sided TWR frame exchange sequence rather than a single synthetic frame.
The current frame progression is:

- `FRAME_POLL`
- `FRAME_RESP`
- `FRAME_FINAL`

The simulator can also emit incomplete DS-TWR exchanges by omitting `RESP` or
`FINAL` frames according to configured missing-frame probabilities. Downstream
packages should interpret the absence of an expected frame within the same
`exchange_seq` as a timeout or incomplete-exchange condition.

Payload injection inputs are accepted per vehicle:

- `/<vehicle_ns>/uwb/tx/payload`

Current simulator examples:

- `/nexus/1/uwb/tx/payload`
- `/nexus/2/uwb/tx/payload`

The per-vehicle topic namespace is intentional.
It keeps the simulator aligned with multi-vehicle ROS namespace structure and allows downstream packages to subscribe vehicle-by-vehicle.

### Simulation-only debug and reference topics

This package may also publish simulation-only helper topics outside the long-term production-facing sensor contract.

Examples include:

- ground-truth distance topics
- LOS debug outputs
- simulator diagnostics
- monitor-oriented helper topics

Downstream packages should not hard-depend on those debug/reference topics unless they are explicitly part of an evaluation-only workflow.

## Message Roles

### `UwbRange`

`UwbRange` is a processed, simulator-oriented measurement output.

It is intended for:

- simulator-side workflows
- rapid downstream prototyping
- dashboard and monitor consumers
- higher-level consumers that want a directly usable range-like measurement

It is not the best candidate for a future real-hardware-facing low-level interface because it already contains simulator-oriented convenience and interpreted fields.

### `RawUWBSignal`

`RawUWBSignal` is the lower-level UWB-facing message.

It is intended for:

- algorithm prototyping at a lower abstraction level
- signal/debug inspection
- experiments that need pre-range-processing style data
- future alignment with a more hardware-like UWB interface

It is the better starting point for future sim-to-real interface evolution.
It should avoid simulator-truth and processed-range fields and instead carry hardware-aligned low-level signal metrics.

The simulator can currently emit either:

- a single synthetic raw frame per ranging cycle (`raw_signal_protocol=single`)
- a simulated DS-TWR exchange sequence (`raw_signal_protocol=ds_twr`)

The DS-TWR mode is the preferred realism path for downstream packages that want
to own state-machine and timestamp interpretation logic.

In DS-TWR mode:

- `exchange_seq` groups `POLL`, `RESP`, and `FINAL` frames that belong to one simulated exchange
- `frame_seq` remains a per-transmitter monotonic counter
- missing `RESP` / `FINAL` frames represent timeout-like protocol failures instead of synthetic timeout frames

## Current Mounting Assumption

The current simulator assumption is:

- one UWB node per vehicle
- configured through `/uwb_simulator/uwb_node`
- mounted at a fixed offset relative to the vehicle body

Current default config shape:

```yaml
uwb_node:
  id: "uwb"
  position:
    x: 0.0
    y: 0.0
    z: 0.0
```

The `id` is currently descriptive only.
Published message identifiers remain vehicle-level (`nexus/1`, `nexus/2`, ...), not node-level.

## Identifier Contract

### `src_id`

`src_id` identifies the transmitting or originating vehicle for the published measurement.

Current convention:

- hierarchical vehicle namespace string such as `nexus/1`, `nexus/2`
- Gazebo model/entity names may remain flat (`nexus1`, `nexus2`) internally, but published ROS message IDs use the hierarchical form

Current single-node interpretation:

- each vehicle currently has one configured onboard UWB node
- `src_id` names the vehicle that owns that node
- the node mount offset affects geometry internally, but not the published identifier

### `dst_id`

`dst_id` identifies the receiving or target vehicle for the published measurement.

Current convention matches `src_id`.

## Timestamp Contract

### `header.stamp`

`header.stamp` is the ROS publication timestamp associated with the simulated measurement event.

Downstream packages should treat it as the measurement timestamp for synchronization and buffering purposes.

Important note:

- this is not currently a hardware capture timestamp copied from a real UWB transceiver
- it is a simulation-time timestamp generated by the simulator

This is acceptable for downstream development as long as consumers treat it consistently as the authoritative timestamp for the simulated measurement.

### `header.frame_id`

Current convention:

- `map`

Downstream packages should not assume this means the measurement itself is spatially expressed in a map frame.
It is currently used as a stable global frame reference marker for the message stream.

If later refinement is needed, this should be changed deliberately and documented as a contract change.

## `UwbRange` Field Contract

`UwbRange` should be treated as a simulator-side processed output.
It is useful and stable for simulation consumers, but it is not the authoritative long-term production-facing contract.

### Stable simulator-consumer fields

These fields are appropriate for simulator consumers:

- `header`
- `src_id`
- `dst_id`
- `distance_3d`
- `sigma_m`
- `rssi`
- `quality`

Interpretation:

- `distance_3d`: the simulated measured range value after channel effects, not pure geometric truth
- `sigma_m`: an uncertainty estimate associated with the measurement
- `rssi`: received signal strength indicator proxy
- `quality`: normalized measurement quality proxy

Downstream simulator consumers may rely on these fields as part of the processed simulation output.

### Simulator convenience fields

These fields are useful and may remain available, but should be treated carefully:

- `distance_2d`
- `los`

Interpretation:

- `distance_2d`: a convenience field mainly useful for formation/debug/analysis workflows
- `los`: simulator-derived visibility diagnostic, not a guaranteed real-hardware field

Guidance:

- downstream packages may use them for simulator evaluation and convenience
- downstream packages should not assume these fields exist in a future real-hardware-facing interface

### Simulation-only or simulator-specific fields

These fields should be treated as simulator-specific diagnostics for now:

- `nlos_bias_mm`
- `fp_index`
- `payload`

Interpretation:

- `nlos_bias_mm`: injected simulator-side NLOS bias summary
- `fp_index`: currently a DW3000-inspired placeholder/diagnostic field, not a stable real-device contract here
- `payload`: simulator convenience channel for embedded sender data, not yet a stable production-facing contract

Guidance:

- downstream packages should not require these fields for sim-to-real architecture
- simulator validation tools may use them freely

## `RawUWBSignal` Field Contract

`RawUWBSignal` should be treated as the message most likely to evolve toward a future hardware-aligned low-level interface.
It is still a simulator message today, but its structure is closer to that goal than `UwbRange`.

### Core low-level fields

These fields are meaningful as low-level signal-oriented outputs:

- `header`
- `src_id`
- `dst_id`
- `toa_ns`
- `tx_timestamp_ps`
- `rx_timestamp_ps`
- `clock_offset_ppm`
- `snr_db`
- `rssi`
- `channel`
- `prf_mhz`
- `first_path_power_dbm`
- `fp_index`
- `sts_quality`
- `frame_seq`
- `exchange_seq`
- `frame_type`
- `frame_payload`
- `cir_real`
- `cir_imag`

These are appropriate for lower-level experimentation and for future hardware-aligned interface work.

`frame_payload` should be interpreted as raw payload bytes carried by the received UWB frame.
It is not application-decoded by this package.
Any message parsing, schema interpretation, or higher-level meaning belongs to downstream packages.
The payload bytes currently come from the latest message published to `/<vehicle_ns>/uwb/tx/payload`.

`tx_timestamp_ps` and `rx_timestamp_ps` are simulator-derived picosecond timestamps intended to be closer to a driver-facing timestamp contract than `toa_ns` alone.
`toa_ns` remains as a convenience field derived from the same simulated timing path.

When DS-TWR mode is active:

- `toa_ns` represents a per-frame one-way arrival estimate
- `frame_type` indicates where the frame belongs in the simulated exchange
- `frame_seq` increments per transmitting vehicle, not globally
- `exchange_seq` is shared by all frames in one simulated DS-TWR transaction
- each vehicle pair currently has a single simulated exchange owner to avoid mirrored duplicate exchanges
- missing `RESP` or `FINAL` frames indicate timeout-like or incomplete exchanges
- `header.stamp` follows the frame TX event time, not only the outer simulator publish cycle

`cir_real` and `cir_imag` are lightweight simulated CIR diagnostics.
They should be treated as approximate low-level signal-shape hints, not as calibrated hardware dumps.

### Core status fields

These fields communicate whether the signal should be treated as usable and in what condition:

- `valid`
- `status_code`

Current intended meaning:

- `valid`: simulator judgement of whether the signal is usable
- `status_code`: compact condition summary for low-level signal state

Current status codes:

- `STATUS_OK`: signal is healthy enough for normal downstream use
- `STATUS_DEGRADED_NLOS`: signal is usable, but likely affected by NLOS propagation
- `STATUS_DEGRADED_OUTLIER`: signal is usable, but likely affected by an outlier-like disturbance
- `STATUS_DEGRADED_WEAK_SIGNAL`: signal is usable, but SNR/power conditions are weak
- `STATUS_INVALID_STS_QUALITY`: STS timing confidence is too poor for the signal to be treated as valid

Current `sts_quality` meaning:

- `sts_quality` is a normalized 0-1 confidence metric for STS-backed RX timestamp trust
- higher values indicate stronger confidence in timestamp quality
- low values indicate the signal may still exist physically, but should be treated cautiously or rejected by downstream logic

### Non-goals for this message

The following should not be carried by `RawUWBSignal` if the goal is to keep it close to a plausible real-hardware-facing interface:

- simulator geometric truth
- already-derived range outputs
- heavy sample-dump diagnostics as part of the default contract

If needed later, such data should live in separate debug-only topics or messages.

## Consumer Guidance

### Recommended default simulator dependency

For most downstream packages, the recommended entry point is:

- subscribe to `/<vehicle_ns>/uwb/range`

This keeps normal simulator integration simple.

### Recommended low-level and future-facing dependency

For analysis, inspection, low-level experimentation, or future hardware-aligned integration work:

- subscribe to `/<vehicle_ns>/uwb/raw_signal`

### Recommended truth dependency policy

Ground-truth or simulator-only fields/topics should be used only for:

- evaluation
- comparison
- debugging
- regression testing

They should not become mandatory dependencies of core downstream algorithms if sim-to-real portability is a goal.

## Current Known Gaps

The current contract is usable, but not yet complete.

The main gaps are:

- no explicit node-level identifier contract in published messages
- no explicit validity/status field in `UwbRange`
- CIR remains a lightweight approximation rather than a hardware-faithful dump
- no formal separation into dedicated production-facing and debug-facing message families

## Near-Term Contract Priorities

The next contract-related improvements should be:

1. keep `UwbRange` clearly positioned as a simulator processed output
2. continue evolving `RawUWBSignal` as the future-facing low-level interface candidate
3. define node-level identifier evolution for future topology support
4. decide whether simulator-only diagnostics should stay inline or move to dedicated debug messages

## Stability Statement

As of now:

- `UwbRange` should be treated as a stable processed simulation message
- `RawUWBSignal` should be treated as the preferred low-level interface candidate for future sim-to-real alignment
- simulator-only truth and diagnostic data must not be assumed to exist in future real-hardware sources

Any deliberate change to these semantics should be reflected in this document before being treated as a stable package-level contract.
