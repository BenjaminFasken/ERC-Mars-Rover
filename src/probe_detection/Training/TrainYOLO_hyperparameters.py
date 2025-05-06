import optuna
from clearml import Task
from ultralytics import YOLO
import torch, json, os

# Clear GPU cache at start
torch.cuda.empty_cache()

# 1) Initialize ClearML
task = Task.init(
    project_name="YOLOv12n Segmentation",
    task_name="tune_mask_mAP50-95_fixed_lr",
    output_uri=True
)

# 2) Best mask mAP tracker
best_mask = {"mask_mAP95": -1.0, "params": None}

# 3) Objective: fixed optimizer & lr0, tune rest
def objective(trial):
    torch.cuda.empty_cache()
    # fixed choices
    optimizer = "SGD"                       # from your sweep  
    lr0 = 0.0055                             # your best lr0  

    # tune remaining HPs
    lrf           = trial.suggest_float("lrf",           0.01, 1.0, log=True)   
    momentum      = trial.suggest_float("momentum",      0.6, 0.98)             
    weight_decay  = trial.suggest_float("weight_decay",  1e-4, 1e-2, log=True)  
    warmup_epochs = trial.suggest_int("warmup_epochs",   0, 5)                  
    warmup_mom    = trial.suggest_float("warmup_momentum",0.5, momentum)         
    cos_lr        = trial.suggest_categorical("cos_lr", [True, False])          

    # 4) Train parameters
    params = dict(
        optimizer=optimizer, lr0=lr0, lrf=lrf, cos_lr=cos_lr,
        momentum=momentum, weight_decay=weight_decay,
        warmup_epochs=warmup_epochs, warmup_momentum=warmup_mom,
        epochs=30, imgsz=640, batch=160
    )

    # 5) Run training
    model = YOLO("yolo11n-seg.pt")
    results = model.train(
        data='/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml',
        cache=True, 
        device=0,
        project=f"runs/trial_{trial.number}", 
        name="train", 
        **params
    )

    # 6) Extract mask mAP50-95
    mask_mAP95 = getattr(results.seg, "map50_95", None)
    if mask_mAP95 is None:
        mask_mAP95 = getattr(results.metrics.seg, "map50_95", 0.0)

    # 7) Log to ClearML
    task.get_logger().report_scalar("mask_mAP50-95", "SGD", trial.number, mask_mAP95)

    # 8) Save best
    if mask_mAP95 > best_mask["mask_mAP95"]:
        best_mask.update({"mask_mAP95": mask_mAP95, "params": params.copy()})
        w = os.path.join(results.save_dir, "weights", "best.pt")
        if os.path.exists(w):
            task.upload_artifact("best_mask_weights", w, metadata=params)

    return mask_mAP95

# 9) Run Optuna study without pruning
study = optuna.create_study(direction="maximize")  # Removed pruner parameter
study.optimize(objective, n_trials=40, gc_after_trial=True)

# 10) Summary artifact
with open("best_mask_mAP95.json","w") as f:
    json.dump(best_mask, f, indent=2)
task.upload_artifact("best_mask_summary", "best_mask_mAP95.json")
print("Done, best:", best_mask)