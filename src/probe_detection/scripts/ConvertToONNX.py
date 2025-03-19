from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("/home/daroe/ERC-Mars-Rover/src/probe_detection/models/many_probes_weights.pt")

# Export the model to ONNX format
model.export(format="onnx")
