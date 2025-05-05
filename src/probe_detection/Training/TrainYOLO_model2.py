import os
import json
import torch
import optuna
from clearml import Task
from ultralytics import YOLO

#Initialize ClearML task
task = Task.init(
    project_name="YOLO11_Segmentation_HPO", 
    task_name="optuna_sgd_segmentation", 
    output_uri=True
)  # logs code, console, scalars, artifacts :contentReference[oaicite:0]{index=0}

#Create Optuna study (maximize mAP)
study = optuna.create_study(direction="maximize")
current_best_mAP = -1.0

def objective(trial):
    global current_best_mAP
    torch.cuda.empty_cache()
    
    #Define search space for core & loss hyperparameters
    params = {
        # fixed optimizer and initial LR
        "optimizer": "SGD",
        "lr0": 0.01,                             # determined externally
        # final LR multiplier
        "lrf": trial.suggest_float("lrf", 0.01, 0.3),              # :contentReference[oaicite:1]{index=1}
        # SGD momentum
        "momentum": trial.suggest_float("momentum", 0.6, 0.98),    # :contentReference[oaicite:2]{index=2}
        # weight decay
        "weight_decay": trial.suggest_float("weight_decay", 0.0, 1e-3, log=True),  # :contentReference[oaicite:3]{index=3}
        # cosine LR scheduler on/off
        "cos_lr": trial.suggest_categorical("cos_lr", [True, False]),
        # warmup schedule
        "warmup_epochs": trial.suggest_int("warmup_epochs", 0, 5),           # :contentReference[oaicite:4]{index=4}
    }
    if params["warmup_epochs"] > 0:
        params["warmup_bias_lr"] = trial.suggest_float("warmup_bias_lr", 0.01, 0.2)    # :contentReference[oaicite:5]{index=5}
        params["warmup_momentum"] = trial.suggest_float("warmup_momentum", 0.0, 0.95) # :contentReference[oaicite:6]{index=6}

    # loss-function gains
    params.update({
        "box": trial.suggest_float("box", 0.01, 10.0),      # :contentReference[oaicite:7]{index=7}
        "cls": trial.suggest_float("cls", 0.01, 10.0),      # :contentReference[oaicite:8]{index=8}
        "mask": trial.suggest_float("mask", 0.01, 10.0),    # common range analogously :contentReference[oaicite:9]{index=9}
        "dfl": trial.suggest_float("dfl", 0.01, 4.0),       # :contentReference[oaicite:10]{index=10}
        "fl_gamma": trial.suggest_float("fl_gamma", 0.0, 5.0),  # :contentReference[oaicite:11]{index=11}
        "fl_alpha": trial.suggest_float("fl_alpha", 0.0, 1.0),  # :contentReference[oaicite:12]{index=12}
        "label_smoothing": trial.suggest_float("label_smoothing", 0.0, 0.2)  # :contentReference[oaicite:13]{index=13}
    })

    # fixed training config
    params["imgsz"] = 640
    params["batch"] = 16
    params["epochs"] = 50

    #Train YOLO11‐seg model with these params
    model = YOLO("yolo11n-seg.pt")  # segmentation‐capable weights :contentReference[oaicite:14]{index=14}
    results = model.train(
        data="path/to/your/data.yaml",
        device=0,
        project="runs/hpo",
        name=f"trial_{trial.number}",
        **params
    )

    # extract final segmentation mAP50 (or box mAP50 if unavailable)
    final_mAP = getattr(results.mask, "map50", None) or results.box.map50

    #Log per-epoch and final mAP to ClearML
    if hasattr(results, "epoch_metrics"):
        for e, m in enumerate(results.epoch_metrics):
            epoch_mAP = (m.get("mask", {}).get("map50") 
                         or m.get("box", {}).get("map50"))
            if epoch_mAP is not None:
                task.get_logger().report_scalar(
                    title=f"trial_{trial.number}_mAP50",
                    series="mAP50",
                    iteration=e,
                    value=epoch_mAP
                )
    task.get_logger().report_scalar(
        title="final_mAP50",
        series="trial",
        iteration=trial.number,
        value=final_mAP
    )
    task.connect(params)

    # upload best weights & params
    if final_mAP > current_best_mAP:
        current_best_mAP = final_mAP
        best_weights = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(best_weights):
            task.upload_artifact(
                name=f"best_weights_trial_{trial.number}",
                artifact_object=best_weights,
                metadata={"mAP50": final_mAP, **params}
            )
            with open("best_params.json","w") as f:
                json.dump({"trial": trial.number, "mAP": final_mAP, "params": params}, f)
            task.upload_artifact("best_params", "best_params.json")

    return final_mAP

#Run the optimization
study.optimize(objective, n_trials=20, gc_after_trial=True)

# optional: upload final best
if current_best_mAP > -1:
    best = study.best_trial
    weights = f"runs/hpo/trial_{best.number}/weights/best.pt"
    if os.path.exists(weights):
        task.upload_artifact("best_weights_final", weights,
                             metadata={"mAP50": best.value, **best.params})
task.close()
