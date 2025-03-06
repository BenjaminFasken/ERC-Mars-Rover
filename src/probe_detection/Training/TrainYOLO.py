from ultralytics import YOLO
import os
import torch


torch.cuda.empty_cache()

print("Script is running from:", os.getcwd())


# Load a pre-trained YOLOv12 model (choose from n, s, m, l, or x versions)
model = YOLO('yolo12s.pt')  # 'yolov12n.pt', 'yolov12m.pt', 'yolov12l.pt', 'yolov12x.pt' are other options

# Train the model on your custom dataset
model.train(data='datasets/Inside/data.yaml', epochs=300, imgsz=640, batch=10)



