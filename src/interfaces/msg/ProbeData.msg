# interfaces/msg/ProbeData.msg

# List of averaged x, y, z positions for each probe
float32[] x
float32[] y
float32[] z

# List of average confidence values
float32[] confidence

# Number of detections that contributed to each probe
int32[] count

# Timestamp of the message
builtin_interfaces/Time stamp
