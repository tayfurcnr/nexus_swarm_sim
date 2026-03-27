# Raw UWB Signal - Hardware-aligned low-level measurement from a DW3000-like transceiver
# This message is intended to resemble device/driver-facing signal metrics rather than
# simulator truth or fully processed ranging output.

# Status codes
uint8 STATUS_OK=0
uint8 STATUS_DEGRADED_NLOS=1
uint8 STATUS_DEGRADED_OUTLIER=2
uint8 STATUS_DEGRADED_WEAK_SIGNAL=3
uint8 STATUS_INVALID_STS_QUALITY=4

# Frame types
uint8 FRAME_UNKNOWN=0
uint8 FRAME_BLINK=1
uint8 FRAME_POLL=2
uint8 FRAME_RESP=3
uint8 FRAME_FINAL=4
uint8 FRAME_DATA=5

Header header

# Identification
string src_id                  # Source drone: iris0, iris1, iris2
string dst_id                  # Destination drone
bool valid                     # True if downstream may use the sample; False means discard is recommended
uint8 status_code              # STATUS_* code describing signal condition; degraded states may still be valid

# Raw Signal Parameters
float32 toa_ns                 # Derived / driver-exposed arrival estimate in nanoseconds, consistent with TX/RX timestamps
uint64 tx_timestamp_ps         # Simulated TX timestamp in picoseconds
uint64 rx_timestamp_ps         # Simulated RX timestamp in picoseconds
float32 clock_offset_ppm       # Relative clock offset / drift estimate in ppm
float32 snr_db                 # Driver-level signal quality metric expressed in dB-like units
float32 rssi                   # Received signal power / RSSI-like metric (dBm)
uint8 channel                  # Configured / simulated PHY channel identifier
uint8 prf_mhz                  # Pulse Repetition Frequency (MHz)

# Signal Quality Indicators
float32 first_path_power_dbm   # First path power estimate (dBm)
uint16 fp_index                # First Path Index (sample number)
float32 sts_quality            # STS-based RX timestamp confidence metric (0-1, higher is better)

# Frame Content
uint16 frame_seq               # Simulated frame sequence number
uint8 frame_type               # FRAME_* identifier for the simulated UWB frame
uint8[] frame_payload          # Raw UWB frame payload bytes (not parsed / not application-decoded)
int16[] cir_real               # Simulated CIR real samples; should match cir_imag length and may be empty
int16[] cir_imag               # Simulated CIR imaginary samples; should match cir_real length and may be empty
