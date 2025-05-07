from clearml import Task
from ultralytics import YOLO
import os
import torch

# Initialize ClearML
task = Task.init(
    project_name="YOLOv12 Training",  # Name of your project in ClearML
    task_name="Mars Yard Training",   # Name of this specific task
    output_uri=True                   # Stores logs and artifacts in ClearML's default location
)

# Clear CUDA cache to free up GPU memory
torch.cuda.empty_cache()

# Print current working directory for debugging
print("Script is running from:", os.getcwd())

# Load a pre-trained YOLOv12 model
model = YOLO('yolo12s.pt')  # Options: 'yolo12n.pt', 'yolo12s.pt', 'yolo12m.pt', 'yolo12l.pt', 'yolo12x.pt'

# Train the model on your custom dataset
model.train(
    data='/home/ucloud/uCloud_training/marsYardData/MarsYard.v2i.yolov12/data.yaml',  # Path to your dataset config
    epochs=20,                         # Number of epochs (increase for better training)
    imgsz=640,                         # Image size
    batch=0.8,                         # Batch size (adjust based on GPU memory)
    device=0,                          # Use GPU (0 is the first GPU)
    cache=True,                        # Cache images for faster training
    plots=True                         # Plot training results
)

# Mark task as completed in ClearML
task.close()
