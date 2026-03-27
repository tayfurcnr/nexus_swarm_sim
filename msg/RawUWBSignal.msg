# Raw UWB Signal - Hardware-aligned low-level measurement from a DW3000-like transceiver
# This message is intended to resemble device/driver-facing signal metrics rather than
# simulator truth or fully processed ranging output.

# Status codes
uint8 STATUS_OK=0
uint8 STATUS_DEGRADED_NLOS=1
uint8 STATUS_DEGRADED_OUTLIER=2
uint8 STATUS_DEGRADED_WEAK_SIGNAL=3
uint8 STATUS_INVALID_STS_QUALITY=4

Header header

# Identification
string src_id                  # Source drone: iris0, iris1, iris2
string dst_id                  # Destination drone
bool valid                     # True if the signal is considered usable by a downstream low-level pipeline
uint8 status_code              # STATUS_* code describing low-level signal condition / degradations

# Raw Signal Parameters
float32 toa_ns                 # Time-of-Arrival (nanoseconds) - PRIMARY measurement
float32 snr_db                 # Driver-level signal quality metric expressed in dB-like units
float32 rssi                   # Received signal power / RSSI-like metric (dBm)
uint8 channel                  # UWB channel (0-15)
uint8 prf_mhz                  # Pulse Repetition Frequency (MHz)

# Signal Quality Indicators
float32 first_path_power_dbm   # First path power estimate (dBm)
uint16 fp_index                # First Path Index (sample number)
float32 sts_quality            # STS-based RX timestamp confidence metric (0-1, higher is better)
