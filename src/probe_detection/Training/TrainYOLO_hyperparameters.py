import os, json, torch, optuna
from clearml import Task
from ultralytics import YOLO

torch.cuda.empty_cache()
task = Task.init(project_name="YOLOv12n Segmentation",
                 task_name="tune_mask_mAP50-95_fixed_lr",
                 output_uri=True)

best_mask = {"mask_mAP95": -1.0, "params": None}

def objective(trial):
    torch.cuda.empty_cache()
    # fixed
    optimizer, lr0 = "SGD", 0.0055

    # suggestions
    lrf    = trial.suggest_float("lrf", 0.01, 1.0, log=True)
    momentum = trial.suggest_float("momentum", 0.6, 0.98)
    weight_decay = trial.suggest_float("weight_decay", 1e-4, 1e-2, log=True)
    warmup_epochs = trial.suggest_int("warmup_epochs", 0, 5)
    warmup_mom    = trial.suggest_float("warmup_momentum", 0.5, momentum)
    cos_lr        = trial.suggest_categorical("cos_lr", [True, False])

    model = YOLO("yolo11n-seg.pt")

    # pruning callback
    def prune_cb(trainer):
        m = trainer.metrics.seg.map50_95 or 0.0
        trial.report(m, trainer.epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    model.add_callback("on_train_epoch_end", prune_cb)

    try:
        res = model.train(
            data='.../rocky_mars.yaml', cache=True, device=0,
            optimizer=optimizer, lr0=lr0, lrf=lrf, cos_lr=cos_lr,
            momentum=momentum, weight_decay=weight_decay,
            warmup_epochs=warmup_epochs, warmup_momentum=warmup_mom,
            epochs=30, imgsz=640, batch=160,
            project=f"runs/trial_{trial.number}", name="train"
        )
    except optuna.exceptions.TrialPruned:
        task.get_logger().report_text(f"Pruned trial {trial.number}")
        raise

    mask_mAP95 = res.metrics.seg.map50_95 or 0.0
    task.get_logger().report_scalar("mask_mAP50-95", "SGD", trial.number, mask_mAP95)
    if mask_mAP95 > best_mask["mask_mAP95"]:
        best_mask.update({"mask_mAP95": mask_mAP95, "params": trial.params})
        w = os.path.join(res.save_dir, "weights", "best.pt")
        if os.path.exists(w):
            task.upload_artifact("best_mask_weights", w, metadata=trial.params)
    return mask_mAP95

study = optuna.create_study(direction="maximize", pruner=optuna.pruners.MedianPruner())
study.optimize(objective, n_trials=40, gc_after_trial=True)

with open("best_mask_mAP95.json","w") as f:
    json.dump(best_mask, f, indent=2)
task.upload_artifact("best_mask_summary", "best_mask_mAP95.json")
print("Done, best:", best_mask)
