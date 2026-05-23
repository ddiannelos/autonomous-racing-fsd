import os
import shutil
import random

# Configuration
source_folder = 'fsoco_dataset'
dataset_dir = "fs_dataset"
val_split = 0.2

def set_dirs():
    for split in ['train', 'val']:
        for dtype in ['images', 'labels']:
            os.makedirs(f"{dataset_dir}/{dtype}/{split}", exist_ok=True)

def move_files():
    # Get list of all images
    images_path = os.path.join(source_folder, "images")
    labels_path = os.path.join(source_folder, "labels")

    # Diverse extensions
    all_files = [f for f in os.listdir(images_path) if f.lower().endswith(('.png', '.jpg', 'jpeg'))]

    # Shuffle to ensure random split
    random.shuffle(all_files)

    # Calculate split index
    split_index = int(len(all_files) * (1 - val_split))
    train_files = all_files[:split_index]
    val_files = all_files[split_index:]

    print(f"Total files: {len(all_files)}")
    print(f"Training: {len(train_files)}, Validation: {len(val_files)}")

    # Function to move pairs
    def move_batch(file_list, split_name):
        for filename in file_list:
            base_name = os.path.splitext(filename)[0]
            label_name = base_name + ".txt"

            src_img = os.path.join(images_path, filename)
            src_lbl = os.path.join(labels_path, label_name)

            dst_img = os.path.join(dataset_dir, "images", split_name, filename)
            dst_lbl = os.path.join(dataset_dir, "labels", split_name, label_name)

            shutil.copy(src_img, dst_img)

            if os.path.exists(src_lbl):
                shutil.copy(src_lbl, dst_lbl)

    print("Moving Training files...")
    move_batch(train_files, 'train')

    print("Moving Validation files...")
    move_batch(val_files, 'val')

    print('Done!')

if __name__ == '__main__':
    set_dirs()
    move_files()
