import torch
from ultralytics import YOLO
from clearml import Task
import os

# Init ClearML task
task = Task.init(
    project_name="YOLOv12n modeltrain",
    task_name="optimizer_showdown_SGD_vs_AdamW",
    output_uri=True
)

torch.cuda.empty_cache()

# Settings
DATA_PATH  = '/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml'
MODEL_PATH = 'yolo11s-seg.pt'
EPOCHS     = 200
BATCH      = 160
IMG_SIZE   = 640

# Best learning rates
optimizers = {
    "SGD":   0.0055,
    "AdamW": 0.0024445076752811906
}

results_dict = {}
for opt_name, lr in optimizers.items():
    print(f"\n⚡ Training with {opt_name} (lr={lr}) for {EPOCHS} epochs")
    torch.cuda.empty_cache()

    model = YOLO(MODEL_PATH)
    results = model.train(
        data=DATA_PATH,
        optimizer=opt_name,
        lr0=lr,
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=BATCH,
        cache=True,
        device=0,
        project=f"runs/showdown_{opt_name}",
        name="train"
    )

    final_map50_95 = getattr(results.seg, 'map50_95', 0.0)

    # log to ClearML
    task.get_logger().report_scalar(
        title="showdown_final_mAP50-95",
        series=opt_name,
        iteration=1,
        value=final_map50_95
    )

    print(f"{opt_name} final mAP50-95 after {EPOCHS} epochs: {final_map50_95:.4f}")
    results_dict[opt_name] = final_map50_95

    # Upload best weights
    weights_path = os.path.join(results.save_dir, "weights", "best.pt")
    if os.path.exists(weights_path):
        task.upload_artifact(
            name=f"best_weights_{opt_name}",
            artifact_object=weights_path,
            metadata={
                "final_map50_95": final_map50_95,
                "optimizer": opt_name,
                "lr": lr
            }
        )

print("\nShowdown complete!")
print("Results:")
for opt, score in results_dict.items():
    print(f"- {opt}: {score:.4f}")
