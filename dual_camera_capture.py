import cv2
import os
import time
import shutil
from datetime import datetime
import threading

# Global base folder
BASE_DIR = "camera"

# Subfolders for both cameras
CAMS = {
    "front_cam_data": "/dev/front_cam",  # Camera index 0
    "back_cam_data":  "/dev/back_cam"  # Camera index 1
}

# Setup folder structure for each camera
def setup_folders():
    for cam_name in CAMS.keys():
        live = os.path.join(BASE_DIR, cam_name, "live_data")
        past = os.path.join(BASE_DIR, cam_name, "past_1_hr_data")
        month = os.path.join(BASE_DIR, cam_name, "monthly_data")
        os.makedirs(live, exist_ok=True)
        os.makedirs(past, exist_ok=True)
        os.makedirs(month, exist_ok=True)

# Capture image from a specific camera
def capture_image(cam_name, cam_index):
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{timestamp}.jpg"

    # Paths
    live_path = os.path.join(BASE_DIR, cam_name, "live_data", filename)
    month_folder = now.strftime("%B").lower()
    date_folder = now.strftime("%d.%m.%Y")
    hour_folder = now.strftime("%I%p").lstrip("0")
    monthly_dir = os.path.join(BASE_DIR, cam_name, "monthly_data", month_folder, date_folder, hour_folder)
    os.makedirs(monthly_dir, exist_ok=True)
    monthly_path = os.path.join(monthly_dir, filename)

    # Capture from specified camera
    cam = cv2.VideoCapture(cam_index)
    ret, frame = cam.read()
    if ret:
        cv2.imwrite(live_path, frame)
        cv2.imwrite(monthly_path, frame)
        print(f"[{cam_name}] Captured: {filename}")
        threading.Timer(600, move_to_past_data, args=[live_path, cam_name]).start()
    cam.release()

# Move image to past_1_hr_data
def move_to_past_data(filepath, cam_name):
    if os.path.exists(filepath):
        filename = os.path.basename(filepath)
        destination = os.path.join(BASE_DIR, cam_name, "past_1_hr_data", filename)
        shutil.move(filepath, destination)
        print(f"[{cam_name}] Moved to past_1_hr_data: {filename}")
        threading.Timer(3600, delete_file, args=[destination]).start()

# Delete image after 1 hour in past_1_hr_data
def delete_file(filepath):
    if os.path.exists(filepath):
        os.remove(filepath)
        print(f"[-] Deleted: {filepath}")

# Setup folders once
setup_folders()

# Main loop
try:
    while True:
        for cam_name, cam_index in CAMS.items():
            capture_image(cam_name, cam_index)
        time.sleep(30)
except KeyboardInterrupt:
    print("Exiting...")
