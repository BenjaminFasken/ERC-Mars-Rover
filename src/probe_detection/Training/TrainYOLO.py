from ultralytics import YOLO
import os
import torch

# Clear CUDA cache to free up GPU memory
torch.cuda.empty_cache()

# Print current working directory for debugging
print("Script is running from:", os.getcwd())

# Load a pre-trained YOLOv12 model (assuming YOLOv12 is supported by Ultralytics)
model = YOLO('yolo12s.pt')  # Options: 'yolo12n.pt', 'yolo12s.pt', 'yolo12m.pt', 'yolo12l.pt', 'yolo12x.pt'

# Train the model on your custom dataset
model.train(
    data='datasets/MarsYard1/data.yaml',  # Path to your dataset config
    epochs=20,                          # Number of epochs (increase for better training)
    imgsz=640,                         # Image size
    batch=12,                          # Batch size (adjust based on GPU memory)
    device=0                           # Use GPU (0 is the first GPU)
)