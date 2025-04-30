import RPi.GPIO as GPIO
import time
import threading
from datetime import datetime

# Constants
ROVER_ID = "Rover_1"
RPI_NO   = "Rpi_1"

# Sensor definitions: (trigger_pin, echo_pin, component_name)
SENSORS = [
    (23, 24, "us1"),
    (25, 27, "us2"),
    (12, 13, "us3"),
    (5,  6,  "us6"),
    (21, 26, "us4"),
    (22, 17, "us5"),
]

# Global GPIO setup
GPIO.setmode(GPIO.BCM)
for trig, echo, _ in SENSORS:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

def monitor_ultrasonic(trig_pin, echo_pin, name):
    """
    Trigger and read one HC-SR04 in parallel.
    Print:
      name:Rover_id,Rpi_no,‹status›,date,time
    where status is:
      - "name detected XX.XXcm" if 2<distance<=40
      - "name detected INF"      if distance>40
      - "name not detected"      on timeout or error
    """
    try:
        # send trigger
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        # wait for echo start
        deadline = time.time() + 0.04
        while GPIO.input(echo_pin) == 0 and time.time() < deadline:
            pulse_start = time.time()
        if time.time() >= deadline:
            raise RuntimeError

        # wait for echo end
        deadline = time.time() + 0.04
        while GPIO.input(echo_pin) == 1 and time.time() < deadline:
            pulse_end = time.time()
        if time.time() >= deadline:
            raise RuntimeError

        # calculate distance
        duration = pulse_end - pulse_start
        distance = round((duration * 34300) / 2, 2)

        if 2 < distance <= 100:
            status = f"{name} detected {distance}cm"
        else:
            status = f"{name} detected INF"

    except:
        status = f"{name} not detected"

    # timestamp
    now = datetime.now()
    date_str = now.strftime("%d/%m/%Y")
    time_str = now.strftime("%I:%M%p").lower()

    # print line
    print(f"{name}:{ROVER_ID},{RPI_NO},{status},{date_str},{time_str}")

if __name__ == "__main__":
    threads = []
    for trig, echo, name in SENSORS:
        t = threading.Thread(target=monitor_ultrasonic, args=(trig, echo, name))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    GPIO.cleanup()
