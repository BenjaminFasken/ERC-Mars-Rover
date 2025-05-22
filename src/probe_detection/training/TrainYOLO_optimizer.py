import optuna
from clearml import Task
from ultralytics import YOLO
import torch
import json
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Clear CUDA cache at program start to avoid OOM issues
torch.cuda.empty_cache()

# 1) Initialize ClearML Task
task = Task.init(
    project_name="YOLOv12n modeltrain",
    task_name="optimizer_comparison_finalfinal", # mikkel would be proud
    output_uri=True
)

# 2) Global for tracking best mAP *per optimizer*
best_mAP_per_optimizer = {
    "SGD": -1.0,
    "AdamW": -1.0
}

best_trial_info = {
    "SGD": None,
    "AdamW": None
}

# 3) Objective: 25 epochs, use final mAP50
def objective(trial):
    global best_mAP_per_optimizer, best_trial_info
    torch.cuda.empty_cache()

    # Choose optimizer and LR
    optimizer = trial.suggest_categorical("optimizer", ["SGD", "AdamW"])
    lr_ranges = {
        "SGD":   (1e-3, 1e-1),   # default 1e-2 centered
        "AdamW": (1e-4, 1e-2),   # default 1e-3 centered
    }
    lr = trial.suggest_float("lr", *lr_ranges[optimizer], log=True)

    # YOLO training params
    params = {
        "optimizer": optimizer,
        "lr0": lr,
        "epochs": 25,
        "imgsz": 640,
        "batch": 160
    }

    # Train model
    model = YOLO('yolo11s-seg.pt')
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True,
        device=0,
        project=f"runs/pilot_trial_{trial.number}",
        name="train",
        **params,
    )

    # Use final mAP50 as objective
    final_map50 = getattr(results.box, 'map50', None)
    if final_map50 is None:
        final_map50 = getattr(results.metrics.box, 'map50', 0.0)

    # Log to ClearML
    task.get_logger().report_scalar(
        title="final_mAP50",
        series=optimizer,
        iteration=trial.number,
        value=final_map50
    )

    # Upload best weights per optimizer
    if final_map50 > best_mAP_per_optimizer[optimizer]:
        best_mAP_per_optimizer[optimizer] = final_map50
        weights_path = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(weights_path):
            task.upload_artifact(
                name=f"best_{optimizer}_weights_trial_{trial.number}",
                artifact_object=weights_path,
                metadata={"final_map50": final_map50, "optimizer": optimizer, "lr": lr}
            )
        # Track best trial info
        best_trial_info[optimizer] = {
            "trial_number": trial.number,
            "final_map50": final_map50,
            "lr": lr
        }

    return final_map50

# 4) Run Optuna study
study = optuna.create_study(direction="maximize")

# Define callback to clear CUDA cache after each trial
def clear_cache_callback(study, trial):
    torch.cuda.empty_cache()

# Run Optuna study
study.optimize(objective, n_trials=20, gc_after_trial=True, callbacks=[clear_cache_callback])

# 5) Local analysis: compare final_map50 per optimizer
sgd_vals = [t.value for t in study.trials if t.params.get("optimizer") == "SGD"]
adamw_vals = [t.value for t in study.trials if t.params.get("optimizer") == "AdamW"]

def median(vals):
    return float(np.median(vals)) if vals else None

def iqr(vals):
    return float(np.percentile(vals,75) - np.percentile(vals,25)) if vals else None

sgd_med = median(sgd_vals);  sgd_iqr = iqr(sgd_vals)
adamw_med = median(adamw_vals); adamw_iqr = iqr(adamw_vals)

print(f"SGD median final mAP50: {sgd_med} ± {sgd_iqr}")
print(f"AdamW median final mAP50: {adamw_med} ± {adamw_iqr}")

# 6) Boxplot
plt.figure()
plt.boxplot([sgd_vals, adamw_vals], labels=["SGD","AdamW"])
plt.ylabel("Final mAP50")
plt.title("Distribution of Final mAP50 by Optimizer (20 trials)")
plt.savefig("boxplot_final_mAP50.png")

# 7) Bar chart
plt.figure()
plt.bar([0,1], [sgd_med, adamw_med], yerr=[sgd_iqr, adamw_iqr], capsize=5)
plt.xticks([0,1], ["SGD","AdamW"])
plt.ylabel("Median Final mAP50 ± IQR")
plt.title("Median Performance by Optimizer")
plt.savefig("bar_median_iqr_final.png")

# 8) Scatter plot LR vs mAP50
df = pd.DataFrame([{'optimizer':t.params['optimizer'], 'lr':t.params['lr'], 'final_map50':t.value} for t in study.trials])
plt.figure()
sns.stripplot(data=df, x='optimizer', y='final_map50', hue='lr', jitter=True)
plt.ylabel('Final mAP50')
plt.title('Final mAP50 vs Learning Rate by Optimizer')
plt.legend(title='lr', bbox_to_anchor=(1,1))
plt.savefig('strip_lr_vs_final_map50.png')

# 9) Summary JSON
summary = {
    "sgd_median_final_map50": sgd_med,
    "sgd_iqr": sgd_iqr,
    "adamw_median_final_map50": adamw_med,
    "adamw_iqr": adamw_iqr,
    "sgd_all": sgd_vals,
    "adamw_all": adamw_vals,
    "best_trials": best_trial_info
}
with open("summary_final_map50.json", "w") as f:
    json.dump(summary, f, indent=2)

# 10) (Optional) log best LR to ClearML scalars
for opt in ["SGD", "AdamW"]:
    info = best_trial_info[opt]
    if info:
        task.get_logger().report_scalar(
            title="best_LR_per_optimizer",
            series=opt,
            iteration=info["trial_number"],
            value=info["lr"]
        )

print("Done: boxplot_final_mAP50.png, bar_median_iqr_final.png, strip_lr_vs_final_map50.png, summary_final_map50.json")
