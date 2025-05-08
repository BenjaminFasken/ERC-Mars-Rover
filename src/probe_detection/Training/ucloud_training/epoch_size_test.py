import torch
from ultralytics import YOLO
import os
import time
import json

def test_epoch_times(model_path, data_config, batch_size, epochs_list, img_size=640, device=0):
    """
    Tests different epoch counts and records training times
    Returns: List of results (epochs, time_sec, error)
    """
    results = []
    
    try:
        if not os.path.exists(model_path):
            return None, f"Model file not found at: {model_path}"

        for epochs in epochs_list:
            model = None
            epoch_time = None
            error_msg = None
            
            try:
                # Initialize fresh model each time
                model = YOLO(model_path)
                
                print(f"\nTesting {epochs} epochs...")
                start_time = time.time()
                
                model.train(
                    data=data_config,
                    epochs=epochs,
                    batch=batch_size,
                    imgsz=img_size,
                    device=device,
                    cache=True,
                    verbose=False,
                    plots=False,
                    val=False,
                    save=False
                )
                
                epoch_time = time.time() - start_time
                print(f"✅ Completed {epochs} epochs in {epoch_time:.1f}s")
                
            except RuntimeError as e:
                error_msg = f"Runtime error: {str(e)}" 
                print(f"❌ Failed at {epochs} epochs: {error_msg}")
            except Exception as e:
                error_msg = f"Unexpected error: {str(e)}"
                print(f"❌ Critical failure at {epochs} epochs: {error_msg}")
                break
            finally:
                # Record results even if failed
                results.append({
                    "epochs": epochs,
                    "time_sec": epoch_time,
                    "error": error_msg
                })
                
                # Cleanup
                if model is not None:
                    del model
                torch.cuda.empty_cache()
                
        return results, None
        
    except Exception as e:
        return None, f"Initialization failed: {str(e)}"

if __name__ == "__main__":
    CONFIG = {
        "model_path": "yolo12n.pt",
        "data_config": "/home/ucloud/ucloud_training/marsYardData/MarsYard.v2i.yolov12/data.yaml",
        "batch_size": 180,  # Use your known working batch size
        "epochs_list": [5, 10, 15, 20, 25, 30, 40, 50, 60, 70],  # Test these epoch counts
        "img_size": 640,
        "device": 0
    }

    print("Starting epoch time tests...")
    results, error = test_epoch_times(**CONFIG)
    
    if results:
        print("\nTest results:")
        for res in results:
            status = "✅ Success" if res['error'] is None else f"❌ Failed: {res['error']}"
            print(f"Epochs: {res['epochs']:2d} | {status} | Time: {res['time_sec'] or 'N/A':.1f}s")
        
        # Save results
        with open("epoch_times2.json", "w") as f:
            json.dump(results, f, indent=2)
        print("\nResults saved to epoch_times2.json")
    else:
        print(f"\nError: {error}")
