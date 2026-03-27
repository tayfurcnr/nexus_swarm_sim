# Hardware Datasheets

Store vendor reference documents here for hardware that informs the simulator's
interface and signal-model design.

Recommended contents:

- UWB module datasheets
- DW3000-related reference manuals
- module pinout or integration guides
- timing or measurement application notes

Recommended naming:

- `vendor_model_datasheet.pdf`
- `vendor_model_register_reference.pdf`
- `vendor_model_app_note_<topic>.pdf`

Notes:

- Keep original vendor PDFs unchanged when possible.
- Prefer adding a short companion note in Markdown if simulator-facing takeaways
  need to be summarized.
- Do not treat documents in this folder as implementation truth by default; any
  design decision taken from them should still be reflected in package docs such
  as `UWB_INTERFACE_CONTRACT.md`.
