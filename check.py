import cv2
import os
import time
import threading
from datetime import datetime
import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import requests

ROVER_ID = "R_001"
RPI_NO = "Rpi_001"
SAVE_DIR = "/home/sabbi/health_check/check_test_folder"
LOCATION_X = ""
LOCATION_Y = ""
LOCATION_Z = ""
POST_URL = "http://localhost:5000/logHealthCheckRPI"
ACTIVITY_URL = "http://localhost:5000/logActivity"
ERROR_URL = "http://localhost:5000/logError"
IMAGE_SIZE_THRESHOLD_KB = 50

SENSORS = [
    (23, 24, "us1"),
    (25, 27, "us2"),
    (12, 13, "us3"),
    (21, 26, "us4"),
]

DIV_RATIO = 11.14

# Global error flag
error_occurred = False
error_messages = []

GPIO.setmode(GPIO.BCM)
for trig, echo, _ in SENSORS:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

def send_health_check(component_name, status, value=None, remarks=""):
    global error_occurred, error_messages
    now = datetime.now().isoformat()
    data = {
        "rover_id": ROVER_ID,
        "rpi_id": RPI_NO,
        "device_id": component_name,
        "check_status": status,
        "check_value": str(value) if value else "",
        "date_time": now,
        "location_x": LOCATION_X,
        "location_y": LOCATION_Y,
        "location_z": LOCATION_Z,
        "remarks": remarks
    }

    try:
        response = requests.post(POST_URL, headers={'Content-Type': 'application/json'}, json=data)
        print(f"✅ Sent [{component_name}] | Status: {response.status_code} | Remarks: {remarks}")

        if status == "1":
            return 
        else:
            error_occurred = True
            error_messages.append(f"{component_name}: {remarks}")

    except requests.exceptions.RequestException as e:
        error_occurred = True
        error_messages.append(f"{component_name}: Request failed - {e}")
        print(f"❌ Failed to send [{component_name}] | Error: {e}")

def create_folder(folder_name):
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

def capture_camera(device_path, component_name):
    create_folder(SAVE_DIR)
    try:
        cap = cv2.VideoCapture(device_path)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                now = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
                filename = os.path.join(SAVE_DIR, f"{component_name}_{now}.jpg")
                cv2.imwrite(filename, frame)

                file_size_kb = os.path.getsize(filename) / 1024
                if file_size_kb > IMAGE_SIZE_THRESHOLD_KB:
                    send_health_check(component_name, "1", filename, "Camera is detected, image > 50KB")
                else:
                    send_health_check(component_name, "2", filename, "Camera is detected, image < 50KB")
            else:
                send_health_check(component_name, "0", "", "Camera is not detected")
        else:
            send_health_check(component_name, "0", "", "Camera is not detected")
    except Exception as e:
        send_health_check(component_name, "ERROR", "", f"Camera error: {e}")

def monitor_cameras():
    threads = []
    threads.append(threading.Thread(target=capture_camera, args=("/dev/front_cam", "cam_f")))
    threads.append(threading.Thread(target=capture_camera, args=("/dev/back_cam", "cam_b")))
    for t in threads:
        t.start()
    for t in threads:
        t.join()

# --- Ultrasonic sensor part (disabled) ---
# def monitor_ultrasonic(trig, echo, name):
#     try:
#         GPIO.output(trig, GPIO.LOW)
#         time.sleep(0.05)
#         GPIO.output(trig, GPIO.HIGH)
#         time.sleep(0.00001)
#         GPIO.output(trig, GPIO.LOW)

#         timeout = time.time() + 0.04
#         while GPIO.input(echo) == 0 and time.time() < timeout:
#             pulse_start = time.time()
#         if time.time() >= timeout:
#             raise RuntimeError("Timeout waiting for echo")

#         timeout = time.time() + 0.04
#         while GPIO.input(echo) == 1 and time.time() < timeout:
#             pulse_end = time.time()
#         if time.time() >= timeout:
#             raise RuntimeError("Timeout waiting for echo to finish")

#         duration = pulse_end - pulse_start
#         distance = round((duration * 34300) / 2, 2)

#         if 2 < distance <= 100:
#             send_health_check(name, "1", distance, f"{name} distance {distance}cm")
#         else:
#             send_health_check(name, "1", "INF", f"{name} out of range")
#     except Exception as e:
#         send_health_check(name, "0", "", f"{name} error: {e}")

def monitor_voltage_sensor(name="voltage_sensor"):
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        chan = AnalogIn(ads, ADS.P0)
        time.sleep(0.1)
        measured = chan.voltage
        actual = round(measured * DIV_RATIO, 2)
        send_health_check(name, "1", actual, "Voltage Read")
    except Exception as e:
        send_health_check(name, "0", "", f"{name} error: {e}")

def log_activity():
    payload = {
        "activity_id": "act-1001",
        "rover_id": ROVER_ID,
        "activity_type": "Health Check",
        "description": "All components passed health check",
        "location_x": LOCATION_X,
        "location_y": LOCATION_Y,
        "location_z": LOCATION_Z,
        "battery_percentage": 0.0,
        "cpu_usage_percentage": 0.0,
        "memory_usage_percentage": 0.0,
        "temperature": 0.0,
        "created_at": datetime.now().isoformat(),
        "created_by": RPI_NO
    }

    try:
        response = requests.post(ACTIVITY_URL, headers={"Content-Type": "application/json"}, json=payload)
        print(f"✅ Activity Log Sent | Status: {response.status_code} | Message: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"❌ Failed to log activity | Error: {e}")

def log_error():
    message = "\n".join(error_messages)
    payload = {
        "activity_id": "act-5647",
        "activity_type": "Sensor Malfunction",
        "created_by": RPI_NO,
        "error_code": "E203",
        "rover_id": ROVER_ID,
        "error_message": message,
        "location_x": LOCATION_X,
        "location_y": LOCATION_Y,
        "location_z": LOCATION_Z,
        "battery_percentage": 0.0,
        "cpu_usage_percentage": 0.0,
        "memory_usage_percentage": 0.0,
        "temperature": 0.0,
        "created_at": datetime.now().isoformat()
    }

    try:
        response = requests.post(ERROR_URL, headers={"Content-Type": "application/json"}, json=payload)
        print(f"✅ Error Log Sent | Status: {response.status_code} | Message: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"❌ Failed to log error | Error: {e}")

def main():
    threads = []

    # --- Skip ultrasonic sensor checks by commenting this block ---
    # for trig, echo, name in SENSORS:
    #     t = threading.Thread(target=monitor_ultrasonic, args=(trig, echo, name))
    #     t.start()
    #     threads.append(t)

    cam_thread = threading.Thread(target=monitor_cameras)
    cam_thread.start()

    voltage_thread = threading.Thread(target=monitor_voltage_sensor)
    voltage_thread.start()

    # for t in threads:
    #     t.join()
    cam_thread.join()
    voltage_thread.join()

    if error_occurred:
        log_error()
    else:
        log_activity()

    GPIO.cleanup()

if __name__ == "__main__":
    main()
