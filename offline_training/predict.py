import os
import shutil
import random
from ultralytics import YOLO

def main():
    # Configuration
    weights_path = 'cone_detection/yolov8n_run2/weights/best.pt'
    val_folder_path = 'fs_dataset/images/val'
    prediction_path = 'predictions'
    num_images = 10

    if not os.path.exists(weights_path):
        print(f"Error: Could not find weights at {weights_path}")
        print("Make sure you have trained the model first")
        return

    # Loading model
    print(f"Loading model: {weights_path}")
    model = YOLO(weights_path)

    # Select random images from val
    all_files = os.listdir(val_folder_path)
    image_files = [f for f in all_files if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    if not image_files:
        print("Error: Could not find image files")
        return

    selected_files = random.sample(image_files, num_images)

    if os.path.exists(prediction_path):
        shutil.rmtree(prediction_path)

    # Predict the selected images
    for i, filename in enumerate(selected_files):
        image_path = os.path.join(val_folder_path, filename)
        results = model.predict(
            source=image_path,
            conf=0.25,
            iou=0.45,
            show=False,
            save=True,
            project='predictions',
            name='run1'
        )

        print(f"[{i+1}/{num_images}] Processed")

    print(f"Success")

if __name__ == "__main__":
    main()