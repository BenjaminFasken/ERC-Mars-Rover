import time
from ultralytics import YOLO

def evaluate_yolo_model(model_path, data_yaml, imgsz=640, batch=16):
    """
    Evaluate a YOLO model using the Ultralytics YOLO package.
    
    Args:
        model_path (str): Path to the YOLO model (e.g., .pt file).
        data_yaml (str): Path to the dataset configuration YAML file.
        imgsz (int): Image size for inference.
        batch (int): Batch size.
        device (str): Device to run evaluation on ('cuda' or 'cpu').
    """
    print(f"Loading model from {model_path}...")
    model = YOLO(model_path)

    print(f"Running validation on data from {data_yaml}...")
    start_time = time.time()
    metrics = model.val(
        data=data_yaml,
        imgsz=imgsz,
        batch=batch,
        device=0,
        verbose=True
    )
    end_time = time.time()

    total_time = end_time - start_time
    num_images = metrics['n']  # Number of images evaluated
    time_per_inference = total_time / num_images if num_images else 0

    print("\n--- Evaluation Results ---")
    print(f"Time per inference: {time_per_inference:.4f} seconds")
    print(f"Precision: {metrics['metrics/precision']:.4f}")
    print(f"Recall: {metrics['metrics/recall']:.4f}")
    print(f"mAP@0.5: {metrics['metrics/mAP_0.5']:.4f}")
    print(f"mAP@0.5:0.95: {metrics['metrics/mAP_0.5:0.95']:.4f}")
    print(f"F1 Score: {metrics['metrics/f1']:.4f}")

if __name__ == "__main__":
    # Example usage
    model_path = "/home/emil/Downloads/best(1).pt"         # Replace with model path
    data_yaml = '/home/ucloud/Training/marsYardData/rocky_mars.v8-big-ahh-dataset-v2.yolov12/data.yaml'
    evaluate_yolo_model(model_path, data_yaml)
