# Standard UWB Range Measurement Message
# Single topic: /uwb/range
# Designed to match DW3000-like hardware interfaces

Header header

# Source and destination drone IDs
string src_id      # e.g. "iris0"
string dst_id      # e.g. "iris1"

# Range measurements
float64 distance_3d   # 3D Euclidean distance (meters)
float64 distance_2d   # 2D horizontal distance (meters), useful for formation
float64 sigma_m       # Standard deviation / uncertainty (meters)

# Environmental/Signal info
bool los              # Line-of-Sight (true if unobstructed)
float32 rssi          # Signal strength (dBm), -100 to 0
float32 quality       # Quality metric (0.0 to 1.0)

# DW3000-like fields
uint16 nlos_bias_mm   # NLOS bias if !los (millimeters)
float32 fp_index      # First path index (optional, 0 if not available)
# Payload data (DW3000 capability)
uint8[] payload       # Custom data from sender (0-127 bytes)
                      # Can contain: drone_id, battery, temperature, state, etc.