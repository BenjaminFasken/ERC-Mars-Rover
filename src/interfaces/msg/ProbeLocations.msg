# ProbeLocations.msg
std_msgs/Header header                   # Standard ROS header
uint32 num_probes                        # Number of probes (N)
float32[] classification_confidence        # Confidence score (0 to 1)
float32[] probes                         # Flattened 3xN array: [x0, y0, z0, x1, y1, z1, ...]