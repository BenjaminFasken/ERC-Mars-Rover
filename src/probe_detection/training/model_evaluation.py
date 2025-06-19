import time
from ultralytics import YOLO

def evaluate_yolo_model(model_path,
                        data_yaml,
                        imgsz: int = 640,
                        batch: int = 16,
                        device: str = 'cpu'):
    """
    Evaluate box and mask (segmentation) metrics on the 'test' split.
    """
    print(f"Loading model from {model_path}...")
    model = YOLO(model_path)

    print(f"Running test evaluation on data from {data_yaml}...")
    start_time = time.time()
    metrics = model.val(
        data=data_yaml,
        split='test',
        imgsz=imgsz,
        batch=batch,
        device=device,
        verbose=True
    )
    total_time = time.time() - start_time
    avg_time = total_time / 286

    # Box (detection) metrics
    box   = metrics.box
    b_pr  = box.p[0]
    b_re  = box.r[0]
    b_f1  = box.f1[0]
    b_map50    = box.map50
    b_map50_95 = box.map() if callable(box.map) else box.map

    # Segmentation (mask) metrics
    seg   = metrics.seg
    s_pr  = seg.p[0]
    s_re  = seg.r[0]
    s_f1  = seg.f1[0]
    s_map50    = seg.map50
    s_map50_95 = seg.map() if callable(seg.map) else seg.map

    print("\n--- Evaluation Results ---")
    print(f"Total eval time:       {total_time:.2f} s")
    print(f"Avg time per image:    {avg_time:.2f} s")
    print("\n[Box Metrics]")
    print(f" Precision:           {b_pr:.4f}")
    print(f" Recall:              {b_re:.4f}")
    print(f" mAP@0.5 (box):       {b_map50:.4f}")
    print(f" mAP@0.5:0.95 (box):  {b_map50_95:.4f}")
    print(f" F1 Score:            {b_f1:.4f}")

    print("\n[Segmentation Metrics]")
    print(f" Precision:           {s_pr:.4f}")
    print(f" Recall:              {s_re:.4f}")
    print(f" mAP@0.5 (mask):      {s_map50:.4f}")
    print(f" mAP@0.5:0.95 (mask): {s_map50_95:.4f}")
    print(f" F1 Score:            {s_f1:.4f}")

if __name__ == "__main__":
    model_path = "src/probe_detection/models/final_models/yolo11s.pt"
    data_yaml  = "src/probe_detection/Training/marsYardData/tempdataset/data.yaml"
    evaluate_yolo_model(model_path, data_yaml, device='cpu')
