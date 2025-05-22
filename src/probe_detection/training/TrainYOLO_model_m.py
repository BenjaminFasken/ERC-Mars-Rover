import torch
import json
import os
from clearml import Task
from ultralytics import YOLO

# Initialize ClearML
task = Task.init(
    project_name="test", #"YOLO11m training",
    task_name="final run (mask mAP50-95)",
    output_uri=True
)

# Fixed hyperparameters for final training
params = {
    "patience": 50,
    "epochs": 300,
    "imgsz": 640,
    "batch": 80,
    "optimizer": "SGD",

    "lr0": 0.003,
    "weight_decay": 0.002457179430445812,
    "momentum": 0.8901479582663367,
    "warmup_momentum": 0.6238178860678709,
    "warmup_epochs": 2,
    "lrf": 0.05676787038842378,
    "cos_lr": True,
}

# Run single final training
def main():
    torch.cuda.empty_cache()

    # Train model
    model = YOLO('yolo11m-seg.pt')
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True,
        device=0,
        project="runs/final_training",
        name="train",
        **params,
    )

    # Extract final mask mAP50-95 (will crash if metric missing)
    mask_mAP095 = results.results_dict["metrics/mAP50-95(M)"]

    # Log final mask mAP50-95
    task.get_logger().report_scalar(
        title="Final_mask_mAP50-95",
        series="Final Run",
        iteration=0,
        value=mask_mAP095
    )

    # Save and upload final weights
    weights_path = os.path.join(results.save_dir, "weights", "best.pt")
    if os.path.exists(weights_path):
        task.upload_artifact(
            name="final_mask_best_weights",
            artifact_object=weights_path,
            metadata={"mask_mAP50-95": mask_mAP095, **params}
        )

    # Save final params locally
    with open("final_mask_params.json", "w") as f:
        json.dump({"mask_mAP50-95": mask_mAP095, "params": params}, f, indent=2)
    task.upload_artifact("final_mask_params", "final_mask_params.json")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        task.get_logger().report_text(f"Error during final training: {e}")
        raise
    finally:
        task.close()
