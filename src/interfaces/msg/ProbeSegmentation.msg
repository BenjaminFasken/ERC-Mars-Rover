# ProbeSegmentation.msg
uint32 id                  # Detection ID
string class_name         # Class name of detected object
float32 confidence        # Confidence score
float32[] box             # Bounding box [x_min, y_min, x_max, y_max]
float32[] mask            # Flattened segmentation mask points
uint32 mask_width         # Width of original mask
uint32 mask_height        # Height of original mask