import torch
from ultralytics import YOLO
from clearml import Task
import numpy as np
import pandas as pd

# ClearML init
task = Task.init(
    project_name="YOLO11n training",
    task_name="lr_sweep",
    output_uri=False
)

# Clear CUDA at start
torch.cuda.empty_cache()

# Define LR sweep (log scale 1e-5 to 1e-1)
lr_values = [float(x) for x in np.logspace(-4, -1, num=20)]

# Store results
results_table = []
best_map50 = -1.0
best_lr = None

# LR sweep loop with enumerate to get integer i
for i, lr in enumerate(lr_values):
    print(f"\n=== Training with LR={lr:.5f} ===")
    torch.cuda.empty_cache()

    # YOLO train params
    params = {
        "optimizer": "SGD",
        "lr0": lr,
        "epochs": 10,
        "imgsz": 640,
        "batch": 160
    }

    # Train model
    model = YOLO('yolo11n-seg.pt')
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True,
        device=0,
        project=f"runs/lr_sweep_lr{lr:.5f}",
        name="train",
        **params,
    )

    # Get final mAP50
    final_map50 = getattr(results.box, 'map50', 0.0)
    print(f"LR {lr:.5f} => final mAP50: {final_map50:.4f}")

    # Save result
    results_table.append((lr, final_map50))

    # Log to ClearML using integer iteration index
    task.get_logger().report_scalar(
        title="lr_sweep_map50",
        series="mAP50",
        iteration=i,
        value=final_map50
    )

    # Track best
    if final_map50 > best_map50:
        best_map50 = final_map50
        best_lr = lr

# Final best summary
print(f"\nBest LR: {best_lr:.5f} with mAP50: {best_map50:.4f}")

# Log table to ClearML
table_text = "| Learning Rate | mAP50 |\n|--------------:|------:|\n"
for lr, map50 in results_table:
    table_text += f"| {lr:.5f}       | {map50:.4f} |\n"

task.get_logger().report_text(f"### LR Sweep Results\n{table_text}")

# Also print table to console
print("\nLR Sweep Results:\n")
print(table_text)

# --- Add CSV saving and ClearML artifact upload ---
# Create DataFrame and save CSV locally
csv_path = "lr_sweep_results.csv"
df = pd.DataFrame(results_table, columns=["lr0","mAP50"])
df.to_csv(csv_path, index=False)
print(f"Saved sweep results to {csv_path}")

# Upload CSV to ClearML
task.upload_artifact(name="lr_sweep_csv", artifact_object=csv_path)
print("Uploaded lr_sweep_results.csv to ClearML")