# Hardware Datasheets

Store vendor reference documents here for hardware that informs the simulator's
interface and signal-model design.

## Current references in this folder

- `DW3000-User-Manual.pdf`
  Primary reference for DW3000 feature semantics, diagnostics, timestamps,
  STS-related behavior, and low-level signal-facing concepts.
- `DW3000 Data Sheet.pdf`
  Product-level capabilities, high-level RF characteristics, supported modes,
  and integration overview.
- `DWM3000TR13.pdf`
  Module-level reference useful for packaged hardware constraints and module
  integration context.

## Recommended next references

- `DW3xxx_QM3xxx_SDK.pdf` or equivalent SDK package notes
  Useful to verify which diagnostics and measurements are realistically exposed
  by software APIs, not just by the chip documentation.
- `DW3000_Errata.pdf`
  Useful for modeling edge cases, degraded states, and non-ideal hardware
  behavior that should influence simulator assumptions.
- Application notes related to:
  - antenna delay calibration
  - TWR error sources
  - production test workflows
  - range maximization and weak-signal behavior

## Recommended contents

- UWB module datasheets
- DW3000-related reference manuals
- module pinout or integration guides
- timing or measurement application notes

## Recommended naming

- `vendor_model_datasheet.pdf`
- `vendor_model_register_reference.pdf`
- `vendor_model_app_note_<topic>.pdf`

## Notes

- Keep original vendor PDFs unchanged when possible.
- Prefer adding a short companion note in Markdown if simulator-facing takeaways
  need to be summarized.
- Do not treat documents in this folder as implementation truth by default; any
  design decision taken from them should still be reflected in package docs such
  as `UWB_INTERFACE_CONTRACT.md`.
- For this package, the most important questions are:
  - Which fields are realistic in `RawUWBSignal`?
  - Which diagnostics are chip-level vs. driver-derived?
  - Which degradations should be modeled in simulation because they affect
    downstream range extraction and localization packages?
