import os
import json
import torch
import optuna
import pandas as pd
from clearml import Task
from ultralytics import YOLO
from optuna.visualization import plot_parallel_coordinate

# Clear GPU cache
torch.cuda.empty_cache()

# 1) Initialize ClearML
task = Task.init(
    project_name="YOLO11l training",
    task_name="hypTuning",
    output_uri=True
)

# 2) Best mask mAP tracker
best_mask = {"mask_mAP95": -1.0, "params": None}

# 3) Create the study (with pruning)
study = optuna.create_study(
    direction="maximize",
    pruner=optuna.pruners.MedianPruner(
        n_startup_trials=10,
        n_warmup_steps=5
    )
)

def objective(trial):
    """Defines one hyperparameter tuning trial."""
    torch.cuda.empty_cache()
    optimizer, lr0 = "SGD", 0.0055

    # Hyperparameter suggestions
    lrf           = trial.suggest_float("lrf",            0.005, 0.8, log=True)
    momentum      = trial.suggest_float("momentum",       0.6,  0.98)
    weight_decay  = trial.suggest_float("weight_decay",   1e-4, 1e-2, log=True)
    warmup_epochs = trial.suggest_int(  "warmup_epochs",    0,   5)
    warmup_mom    = trial.suggest_float("warmup_momentum",0.5, momentum)
    cos_lr        = trial.suggest_categorical("cos_lr", [True, False])

    # Load model
    model = YOLO("yolo11s-seg.pt")

    # Pruning callback
    def prune_cb(trainer):
        m = trainer.metrics.get("metrics/mAP50-95(M)", 0.0)
        trial.report(m, trainer.epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    model.add_callback("on_fit_epoch_end", prune_cb)

    # Train (may raise TrialPruned)
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True, device=0,
        optimizer=optimizer, lr0=lr0, lrf=lrf, cos_lr=cos_lr,
        momentum=momentum, weight_decay=weight_decay,
        warmup_epochs=warmup_epochs, warmup_momentum=warmup_mom,
        epochs=20, imgsz=640, batch=160,
        project=f"runs/trial_{trial.number}", name="train"
    )

    # Extract final mask mAP50-95
    res_dict   = results.results_dict if hasattr(results, "results_dict") else results
    mask_mAP95 = res_dict.get("metrics/mAP50-95(M)", 0.0)

    # Track best and upload weights if new best
    if mask_mAP95 > best_mask["mask_mAP95"]:
        best_mask.update({"mask_mAP95": mask_mAP95, "params": trial.params})
        w = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(w):
            task.upload_artifact("best_mask_weights", w, metadata=trial.params)

    return mask_mAP95

def save_trial_callback(study, trial):
    """Post-trial callback: append the completed trial’s row to CSV."""
    # Get the full dataframe (now trial.state, value, datetime, duration are all set)
    df = study.trials_dataframe()
    row = df[df["number"] == trial.number]

    # Append or create
    if os.path.exists("optuna_trials.csv"):
        row.to_csv("optuna_trials.csv", mode="a", header=False, index=False)
    else:
        row.to_csv("optuna_trials.csv", mode="w", header=True, index=False)

# 4) Run the optimization with our callback
study.optimize(
    objective,
    n_trials=70,
    gc_after_trial=True,
    callbacks=[save_trial_callback]
)

# 5) Save the best result summary
with open("best_mask_mAP95.json", "w") as f:
    json.dump(best_mask, f, indent=2)
task.upload_artifact("best_mask_summary", "best_mask_mAP95.json")

# 6) Final export of ALL trials
df_final = study.trials_dataframe()
df_final.to_csv("optuna_trials_final.csv", index=False)

# 7) Parallel-coordinate plot
fig = plot_parallel_coordinate(study)
fig.write_image("parallel_coordinate_plot.png")

print("Done, best:", best_mask)
