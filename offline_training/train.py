from ultralytics import YOLO

def main():
    # Load model yolov8n.pt
    print("Loading model...")
    model = YOLO('yolov8n.pt')

    # Train  the model
    print('Starting training....')
    results = model.train(
        data='fs_dataset/data.yaml',
        epochs=300,
        patience=20,
        imgsz=1024,
        batch=4,
        workers=4,
        device=0,
        project='cone_detection',
        name='yolov8n_run2',
        exist_ok=True,
        pretrained=True,
        optimizer='auto',
        verbose=True,
        cache=True
    )

    print('Training finished. Validating...')
    metrics = model.val()

    print(f"Mean Average Percision: {metrics.box.map50}")

if __name__ == "__main__":
    main()