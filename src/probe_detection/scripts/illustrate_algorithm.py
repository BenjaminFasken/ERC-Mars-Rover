#!/usr/bin/env python3
import cv2
import numpy as np
import os
from ultralytics import YOLO
from typing import Tuple, List
import uuid
from ament_index_python.packages import get_package_share_directory

class SegmentationProcessor:
    def __init__(self, model_path: str, save_path: str = "/tmp/probe_detection"):
        """
        Initialize the segmentation processor with a YOLO model and save path.
        
        Args:
            model_path (str): Path to the YOLO model file
            save_path (str): Directory to save output images
        """
        self.confidence_threshold = 0.5
        self.save_path = save_path
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_path, exist_ok=True)
        
        # Load YOLO model
        try:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found at: {model_path}")
            self.model = YOLO(model_path)
            print(f"Loaded YOLO model from {model_path}")
        except Exception as e:
            print(f"Failed to initialize model: {e}")
            raise

    def infer_yolo(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray], np.ndarray]:
        """
        Run YOLO inference on the input RGB image.
        
        Args:
            rgb_image (np.ndarray): Input RGB image
            
        Returns:
            Tuple containing boxes, masks, and confidences
        """
        probe_boxes = []
        probe_masks = []
        probe_confidences = []

        try:
            results = self.model.predict(
                source=rgb_image,
                imgsz=640,
                conf=self.confidence_threshold,
                device=0
            )
            
            if results[0].boxes is not None and results[0].masks is not None:
                for i, (box, mask) in enumerate(zip(results[0].boxes, results[0].masks)):
                    try:
                        box_xyxy = box.xyxy[0].cpu().numpy()
                        mask_data = mask.data[0].cpu().numpy()
                        mask_data = cv2.resize(mask_data, (rgb_image.shape[1], rgb_image.shape[0]))
                        
                        probe_boxes.append(box_xyxy)
                        probe_masks.append(mask_data)
                        probe_confidences.append(float(box.conf.cpu()))
                        
                    except Exception as e:
                        print(f"Error processing detection {i}: {e}")
                        continue

            probe_boxes = np.array(probe_boxes, dtype=np.float32) if probe_boxes else np.array([], dtype=np.float32).reshape(0, 4)
            probe_confidences = np.array(probe_confidences, dtype=np.float32) if probe_confidences else np.array([], dtype=np.float32)

        except Exception as e:
            print(f"Error in YOLO inference: {e}")

        return probe_boxes, probe_masks, probe_confidences

    def process_image(self, image_path: str) -> None:
        """
        Process a single RGB image and save three output images.
        
        Args:
            image_path (str): Path to the input RGB image
        """
        try:
            # Read and save original image
            rgb_image = cv2.imread(image_path)
            if rgb_image is None:
                raise ValueError(f"Failed to load image from {image_path}")
            
            original_image_path = os.path.join(self.save_path, f"original_image_{uuid.uuid4()}.png")
            cv2.imwrite(original_image_path, rgb_image)
            print(f"Saved original image to {original_image_path}")

            # Run YOLO inference
            probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)
            
            # Create combined binary mask
            combined_mask = np.zeros((rgb_image.shape[0], rgb_image.shape[1]), dtype=np.uint8)
            probe_locations = []
            
            for i, mask in enumerate(probe_masks):
                try:
                    # Normalize mask to binary (0 or 1)
                    binary_mask = (mask > 0.5).astype(np.uint8)
                    combined_mask = np.maximum(combined_mask, binary_mask)
                    
                    # Compute centroid
                    y, x = np.where(binary_mask == 1)
                    if x.size > 0 and y.size > 0:
                        centroid_x = int(np.mean(x))
                        centroid_y = int(np.mean(y))
                        probe_locations.append({
                            "centroid_x": centroid_x,
                            "centroid_y": centroid_y,
                            "confidence": probe_confidences[i]
                        })
                except Exception as e:
                    print(f"Error processing mask {i}: {e}")
                    continue

            # Save binary mask as PNG
            binary_mask_image = (combined_mask * 255).astype(np.uint8)
            mask_image_path = os.path.join(self.save_path, f"binary_mask_{uuid.uuid4()}.png")
            cv2.imwrite(mask_image_path, binary_mask_image)
            print(f"Saved binary mask to {mask_image_path}")

            # Create final image with probe centers
            final_image = rgb_image.copy()
            for loc in probe_locations:
                centroid_x = loc["centroid_x"]
                centroid_y = loc["centroid_y"]
                confidence = loc["confidence"]
                cv2.circle(final_image, (centroid_x, centroid_y), 3, (0, 0, 255), -1)  # Red dot
                cv2.putText(final_image, f"conf={confidence:.2f}", 
                           (centroid_x + 5, centroid_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (255, 255, 255), 1)

            # Save final image
            final_image_path = os.path.join(self.save_path, f"final_image_{uuid.uuid4()}.png")
            cv2.imwrite(final_image_path, final_image)
            print(f"Saved final image to {final_image_path}")

        except Exception as e:
            print(f"Error processing image: {e}")

def main():
    # Configure paths
    package_name = 'probe_detection'
    package_share_dir = get_package_share_directory(package_name)
    model_path = os.path.join(package_share_dir, 'models', 'probe_segmentation_v2.pt')
    save_path = "/home/daroe/ERC-Mars-Rover/datasets/raw_images/Illustration_image"  # Modify this to your desired save path
    image_path = "/home/daroe/ERC-Mars-Rover/datasets/raw_images/Illustration_image/Original.jpg"  # Modify this to your input image path

    try:
        processor = SegmentationProcessor(model_path=model_path, save_path=save_path)
        processor.process_image(image_path=image_path)
    except Exception as e:
        print(f"Error in main: {e}")

if __name__ == '__main__':
    main()