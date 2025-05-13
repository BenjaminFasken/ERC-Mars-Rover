import os
import json
import torch
import optuna
from clearml import Task
from ultralytics import YOLO
from optuna.visualization import plot_parallel_coordinate

# Clear GPU cache
torch.cuda.empty_cache()

# 1) Initialize ClearML
task = Task.init(
    project_name="test",
    task_name="hypTuning",
    output_uri=True
)

# 2) Best mask mAP tracker
best_mask = {"mask_mAP95": -1.0, "params": None}

def objective(trial):
    torch.cuda.empty_cache()
    optimizer, lr0 = "SGD", 0.0055

    # Hyperparameter suggestions
    lrf           = trial.suggest_float("lrf",            0.005, 0.8, log=True)
    momentum      = trial.suggest_float("momentum",       0.6,  0.98)
    weight_decay  = trial.suggest_float("weight_decay",   1e-4, 1e-2, log=True)
    warmup_epochs = trial.suggest_int("warmup_epochs",    0,   5)
    warmup_mom    = trial.suggest_float("warmup_momentum",0.5, momentum)
    cos_lr        = trial.suggest_categorical("cos_lr", [True, False])

    model = YOLO("yolo11s-seg.pt")

    # 3) Pruning callback on on_fit_epoch_end
    def prune_cb(trainer):
        # Use dict access on trainer.metrics
        m = trainer.metrics.get("metrics/mAP50-95(M)", 0.0)
        trial.report(m, trainer.epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    model.add_callback("on_fit_epoch_end", prune_cb)

    try:
        results = model.train(
            data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
            cache=True, device=0,
            optimizer=optimizer, lr0=lr0, lrf=lrf, cos_lr=cos_lr,
            momentum=momentum, weight_decay=weight_decay,
            warmup_epochs=warmup_epochs, warmup_momentum=warmup_mom,
            epochs=20, imgsz=640, batch=160,
            project=f"runs/trial_{trial.number}", name="train"
        )
    except optuna.exceptions.TrialPruned:
        task.get_logger().report_text(f"Pruned trial {trial.number}")
        raise

    # Convert the SegmentMetrics return into a flat dict
    res_dict = results.results_dict if hasattr(results, "results_dict") else results

    # 4) Final mask mAP50-95
    mask_mAP95 = res_dict.get("metrics/mAP50-95(M)", 0.0)

    '''
    # 5) Correct ClearML logging (ensure iteration is int, value is float)
    task.get_logger().report_scalar(
        title="mask_mAP50-95",
        series="SGD",
        iteration=int(trial.number),
        value=float(mask_mAP95)
    )
    '''

    # 5) Track best
    if mask_mAP95 > best_mask["mask_mAP95"]:
        best_mask.update({"mask_mAP95": mask_mAP95, "params": trial.params})
        w = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(w):
            task.upload_artifact("best_mask_weights", w, metadata=trial.params)

    return mask_mAP95

# 6) Run study
study = optuna.create_study(direction="maximize", pruner=optuna.pruners.MedianPruner(n_startup_trials=15, n_warmup_steps=5))

study.optimize(objective, n_trials=70, gc_after_trial=True)

# 7) Save summary
with open("best_mask_mAP95.json", "w") as f:
    json.dump(best_mask, f, indent=2)
task.upload_artifact("best_mask_summary", "best_mask_mAP95.json")

# 8) Save trials & parallel-coordinate plot
df = study.trials_dataframe()
df.to_csv("optuna_trials.csv", index=False)
fig = plot_parallel_coordinate(study)
fig.write_image("parallel_coordinate_plot.png")

print("Done, best:", best_mask)
