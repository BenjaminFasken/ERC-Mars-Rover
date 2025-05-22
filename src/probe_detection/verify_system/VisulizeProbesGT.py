import json
import cv2
from pathlib import Path

def visualize_gt_probes():
    # Directories
    base_dir = Path("~/ERC-Mars-Rover/src/probe_detection/test").expanduser()
    data_dir = base_dir / "data"
    annotation_dir = base_dir / "annotations"

    # Image dimensions
    image_width = 640
    image_height = 360

    # Load annotation and image files
    annotation_files = sorted(annotation_dir.glob("image_*_annotation.json"))
    image_files = sorted(data_dir.glob("image_*.png"))

    if not annotation_files or not image_files:
        print(f"No annotation or image files found in {annotation_dir} or {data_dir}")
        return

    # Ensure matching files
    if len(annotation_files) != len(image_files):
        print(f"Mismatch: Found {len(annotation_files)} annotations and {len(image_files)} images")
        return

    # Process each image and annotation pair
    for ann_path, img_path in zip(annotation_files, image_files):
        image_id = img_path.stem  # e.g., image_001
        if ann_path.stem != f"{image_id}_annotation":
            print(f"Mismatch: Annotation {ann_path.stem} does not match image {image_id}")
            continue

        # Load annotation
        with open(ann_path, 'r') as f:
            annotation = json.load(f)

        # Load image
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Failed to read image: {img_path}")
            continue

        # Draw GT probes
        for probe_data in annotation.get("probes", []):
            identifier = probe_data.get("identifier", "Unknown")
            bbox = probe_data.get("bounding_box", [0, 0, 0, 0])  # [center_x, center_y, width, height], normalized
            center_x = int(bbox[0] * image_width)  # Convert to pixels
            center_y = int(bbox[1] * image_height)

            # Draw red circle at center
            cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
            # Label with GT<identifier>
            cv2.putText(img, f"GT{identifier}", (center_x + 10, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            # Add sign of y axis, to see to the right or left (+ or -)
            ground_plane_y = probe_data.get("ground_truth_coordinates_ground_plane", {}).get("y", 1)
            sign = "+" if ground_plane_y > 0 else "-"
            print(f"Ground plane y: {ground_plane_y}, Sign: {sign}")
            
            cv2.putText(img, f"y: {sign}", (center_x + 10, center_y + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Show image
        cv2.imshow(f"GT Probes - {image_id}", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print(f"Displayed {len(annotation.get('probes', []))} GT probes for {image_id}")

if __name__ == "__main__":
    visualize_gt_probes()