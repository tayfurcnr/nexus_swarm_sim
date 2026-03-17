# Raw UWB Signal - Low-level measurement from DW3000-like transceiver
# This message represents raw signal parameters BEFORE signal processing
# Your algorithm should extract ToA and convert to distance

Header header

# Identification
string src_id                  # Source drone: iris0, iris1, iris2
string dst_id                  # Destination drone

# Raw Signal Parameters
float32 toa_ns                 # Time-of-Arrival (nanoseconds) - PRIMARY measurement
float32 snr_db                 # Signal-to-Noise Ratio (dB)
float32 rssi                   # Received Signal Strength Indicator (dBm)
uint8 channel                  # UWB channel (0-15)
uint8 prf                      # Pulse Repetition Frequency (MHz)

# Signal Quality Indicators
float32 fppl                   # First Path Power Level (dBm)
uint16 fp_index                # First Path Index (sample number)
uint8 n_paths                  # Number of multipath components detected
float32 sts_quality            # STS (Scrambled Timestamp) quality (0-1)

# Ground Truth (DEBUG ONLY - remove in real system)
float32 true_distance_m        # True distance (Gazebo ground truth, for validation)

# Raw Signal Samples (optional - high bandwidth)
float32[] signal_samples       # I/Q samples or amplitude envelope (optional)
