import torch
from ultralytics import YOLO
import os
import gc

def find_max_batch_size(model_path, data_config, img_size=640, device=0):
    """
    Finds the maximum viable batch size for your GPU
    Returns: (max_batch_size, error_message)
    """
    try:
        if not os.path.exists(model_path):
            return None, f"Model file not found at: {model_path}"

        batch_sizes = [220, 224, 228, 232, 236, 240, 244, 248, 250, 252, 256]  # Test from smallest to largest
        max_successful_batch = 0

        for batch in batch_sizes:
            model = None
            try:
                # Create new model instance for each test
                model = YOLO(model_path)
                print(f"\nTesting batch size: {batch}")
                
                # Test with minimal settings
                results = model.train(
                    data=data_config,
                    epochs=1,
                    imgsz=img_size,
                    batch=batch,
                    device=device,
                    cache=False,
                    verbose=False,
                    plots=False,
                    val=False,
                    save=False
                )
                
                print(f"✅ Success with batch size {batch}")
                max_successful_batch = batch  # Update max successful batch
                
            except RuntimeError as e:
                if 'out of memory' in str(e).lower():
                    print(f"❌ OOM with batch size {batch}")
                else:
                    return None, str(e)
            finally:
                # Cleanup after each attempt
                if model is not None:
                    del model
                torch.cuda.empty_cache()
                gc.collect()

        if max_successful_batch > 0:
            return max_successful_batch, None
        else:
            return None, "No viable batch size found"
            
    except Exception as e:
        return None, f"Initialization failed: {str(e)}"

if __name__ == "__main__":
    CONFIG = {
        "model_path": "yolo12n.pt",
        "data_config": "/home/ucloud/ucloud_training/marsYardData/MarsYard.v2i.yolov12/data.yaml",
        "img_size": 640,
        "device": 0
    }

    print("Starting batch size test...")
    max_batch, error = find_max_batch_size(**CONFIG)
    
    if max_batch:
        print(f"\nMaximum viable batch size: {max_batch}")
        print(f"Recommended training batch size: {int(max_batch * 0.8)} (80% of max)")
    else:
        print(f"\nError: {error}")
