Header header

# System Status
bool system_ok
string system_mode
string[] active_errors
string[] active_warnings

# Connection Status
bool fcu_connected
time last_heartbeat
float32 connection_quality

# Flight status
bool armed
string flight_mode
float32 flight_time

# Battery status
float32 voltage
float32 current
float32 remaining_percentage
float32 remaining_time
uint8 battery_status
string power_status

# Position & Navigation
bool gps_ok
uint8 gps_satellites
float32 gps_hdop
bool position_ok
float32[3] position_xyz
float32[3] velocity_xyz
float32[3] attitude_rpy
float32 position_accuracy
float32 altitude_agl

# Mission Status
string mission_state
float32 mission_progress
string current_waypoint
uint32 waypoints_remaining

# Performance Metrics
float32 cpu_usage
float32 memory_usage
float32 disk_usage
float32 network_bandwidth
float32 loop_rate

# Custom Fields
string[] custom_status_names
float32[] custom_status_values
