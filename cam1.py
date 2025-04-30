import cv2
import os
import time
import threading
from datetime import datetime

# Constants
ROVER_ID = "Rover_1"
RPI_NO = "Rpi_1"
SAVE_DIR = "test"
DELETE_AFTER_SECONDS = 15 * 60  # 15 minutes

def create_folder(folder_name):
    """Create a folder if it does not exist."""
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

def delete_after_delay(file_path, delay):
    """Delete a file after a certain delay in seconds."""
    def delete_file():
        time.sleep(delay)
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"Deleted old image: {file_path}")
    threading.Thread(target=delete_file, daemon=True).start()

def capture_camera(device_path, component_name):
    """Capture an image from a camera and handle saving and deletion."""
    create_folder(SAVE_DIR)
    try:
        cap = cv2.VideoCapture(device_path)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                now = datetime.now()
                timestamp = now.strftime("%d-%m-%Y_%H-%M-%S")  # Safe format for filename
                image_filename = os.path.join(SAVE_DIR, f"{component_name}_{timestamp}.jpg")
                cv2.imwrite(image_filename, frame)
                delete_after_delay(image_filename, DELETE_AFTER_SECONDS)
                comp_status = f"{component_name} detected"
            else:
                image_filename = "No Image Captured"
                comp_status = f"{component_name} not responding"
        else:
            image_filename = "No Image Captured"
            comp_status = f"{component_name} not detected"
    except Exception as e:
        image_filename = "Error"
        comp_status = f"{component_name} error {str(e)}"

    now = datetime.now()
    date_str = now.strftime("%d/%m/%Y")
    time_str = now.strftime("%I:%M%p").lower()

    output = f"{component_name}:{ROVER_ID},{RPI_NO},{comp_status},{date_str},{time_str},{image_filename}"
    print(output)

def monitor_cameras():
    """Monitor both cameras in parallel threads."""
    front_thread = threading.Thread(target=capture_camera, args=("/dev/front_cam", "cam_f"))
    back_thread = threading.Thread(target=capture_camera, args=("/dev/back_cam", "cam_b"))

    front_thread.start()
    back_thread.start()

    front_thread.join()
    back_thread.join()

if __name__ == "__main__":
    monitor_cameras()
