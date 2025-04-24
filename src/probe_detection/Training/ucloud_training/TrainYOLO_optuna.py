import optuna
from clearml import Task
from ultralytics import YOLO
import torch
import json
import os

# Initialize ClearML
task = Task.init(
    project_name="YOLOv12n hyperparameter setup",
    task_name="Optuna Training (Fixed Error)",
    output_uri=True
)

# Optuna study
study = optuna.create_study(direction="maximize")

# Manual tracking of best mAP
current_best_mAP = -1  # Initialize to impossible value

def objective(trial):
    global current_best_mAP
    
    torch.cuda.empty_cache()
    
    # Hyperparameters
    epochs = trial.suggest_int("epochs", 5, 7)
    batch = trial.suggest_int("batch", 60, 80)

    # Train
    model = YOLO('yolo12n.pt')
    results = model.train(
        data='/home/ucloud/uCloud_training/marsYardData/MarsYard.v2i.yolov12/data.yaml',
        epochs=epochs,
        batch=batch,
        device=0,
        project=f"runs/trial_{trial.number}",
        name="train"
    )

    mAP = results.box.map50

    # Log metrics/params
    task.get_logger().report_scalar("mAP_50", "Trial", iteration=trial.number, value=mAP)
    task.connect(trial.params)

    # Save weights if improved (without using study.best_trial)
    if mAP > current_best_mAP:
        current_best_mAP = mAP
        
        weights_path = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(weights_path):
            task.upload_artifact(
                name=f"best_weights_trial_{trial.number}",
                artifact_object=weights_path,
                metadata={"mAP_50": mAP, **trial.params}
            )
            
            # Save params
            with open("best_params.json", "w") as f:
                json.dump({"mAP": mAP, "params": trial.params}, f)
            task.upload_artifact("best_params", "best_params.json")

    return mAP

# Run optimization
try:
    study.optimize(objective, n_trials=10, gc_after_trial=True)
    
    # Final upload (redundant safety)
    if current_best_mAP > -1:  # If at least one trial succeeded
        best_trial = study.best_trial  # Now safe to access
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
