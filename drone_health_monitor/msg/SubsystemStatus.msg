# File: msg/SubsystemStatus.msg
string subystem_name
uint8 status_level
string message
float32[] values

# Predefined status levels
uint8 OK=0
uint8 WARNING=1
uint8 ERROR=2
uint8 CRITICAL=3

