import optuna
from clearml import Task
from ultralytics import YOLO
import torch
import json
import os

# Initialize ClearML
task = Task.init(
    project_name="YOLOv12n modeltrain",
    task_name="hyperparameter_tuning",
    output_uri=True
)

# Optuna study
study = optuna.create_study(direction="maximize")

# Manual tracking of best mAP (final mAP per trial)
current_best_mAP = -1  # Initialize to impossible value

def objective(trial):
    global current_best_mAP
    
    torch.cuda.empty_cache()
    
    # Hyperparameters
    optimizer = trial.suggest_categorical("optimizer", ["SGD", "AdamW"])
    
    # Set ranges depending on the optimizer
    lr0_range = {
        "SGD": (3e-3, 3e-2),    # 0.003 to 0.03
        "AdamW": (1e-4, 1e-3)   # 0.0001 to 0.001
    }
    
    # Fix: Remove duplicate weight_decay key.
    params = {
        "optimizer": optimizer,
        "lr0": trial.suggest_float("lr0", *lr0_range[optimizer], log=True),
        # Use a single weight_decay parameter – adjust range as needed:
        "weight_decay": trial.suggest_float("weight_decay", 1e-5, 1e-3, log=True),
        "cos_lr": trial.suggest_categorical("cos_lr", [True, False]),
        "warmup_epochs": trial.suggest_int("warmup_epochs", 3, 5),
        "overlap_mask": False,
        "mask_ratio": 2,
        "imgsz": 640,
        "batch": 180,   # Using ~80% of the tested absolute max
        "epochs": 10,   # Adjust as needed 25 for now
    }
    
    if optimizer == "SGD":
        params["momentum"] = trial.suggest_float("momentum", 0.9, 0.98)
        
    if params["cos_lr"]:
        params["lrf"] = trial.suggest_float("lrf", 0.01, 0.3)
    
    if params["warmup_epochs"] > 0:
        params["warmup_bias_lr"] = trial.suggest_float("warmup_bias_lr", 0.01, 0.2)
        if optimizer == "SGD":
            params["warmup_momentum"] = trial.suggest_float("warmup_momentum", 0.8, 0.95)
    
    # Train model using YOLO
    model = YOLO('yolo12n.pt')
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True,
        device=0,
        project=f"runs/trial_{trial.number}",
        name="train",
        **params,
    )
    
    # Assume results.box.map50 gives the final mAP50.
    final_mAP = results.box.map50

    # Log a convergence curve for this trial.
    # (Assume results has an attribute 'epoch_metrics' that is a list where each item is a dict
    # with the epoch’s performance; for example, each element's "box" key is a dict with "map50")
    if hasattr(results, 'epoch_metrics'):
        for epoch, epoch_metrics in enumerate(results.epoch_metrics):
            # Adjust the key access below as necessary. For instance, if each epoch_metrics item is a dict 
            # with a key 'box' that itself is a dict with the key 'map50':
            epoch_mAP = epoch_metrics.get('box', {}).get('map50', None)
            if epoch_mAP is not None:
                task.get_logger().report_scalar(
                    title=f"Trial_{trial.number}_mAP50_convergence",
                    series="mAP50",
                    iteration=epoch,
                    value=epoch_mAP
                )
    else:
        # Fallback: if epoch-level metrics are not available, log only the final mAP (no slider)
        task.get_logger().report_scalar(
            title=f"Trial_{trial.number}_mAP50_convergence",
            series="mAP50",
            iteration=0,
            value=final_mAP
        )
    
    # Log the final mAP for this trial separately (if desired)
    task.get_logger().report_scalar(
        title="Final_mAP_50",
        series="Trial",
        iteration=trial.number,
        value=final_mAP
    )
    
    # Connect hyperparameters so that they appear in the ClearML UI
    task.connect(trial.params)
    
    # Save weights if improved
    if final_mAP > current_best_mAP:
        current_best_mAP = final_mAP
        weights_path = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(weights_path):
            task.upload_artifact(
                name=f"best_weights_trial_{trial.number}",
                artifact_object=weights_path,
                metadata={"mAP_50": final_mAP, **trial.params}
            )
            # Save parameters locally and upload the file
            with open("best_params.json", "w") as f:
                json.dump({"trial num": trial.number, "mAP": final_mAP, "params": trial.params}, f)
            task.upload_artifact("best_params", "best_params.json")
    
    return final_mAP

# Run optimization
try:
    study.optimize(objective, n_trials=10, gc_after_trial=True)
    if current_best_mAP > -1:  # If at least one trial succeeded
        best_trial = study.best_trial
        best_weights_path = f"runs/trial_{best_trial.number}/train/weights/best.pt"
        if os.path.exists(best_weights_path):
            task.upload_artifact(
                "best_weights_final",
                best_weights_path,
                metadata={"mAP_50": best_trial.value, **best_trial.params}
            )
except Exception as e:
    task.get_logger().report_text(f"Error: {str(e)}")
    raise
finally:
    task.close()
