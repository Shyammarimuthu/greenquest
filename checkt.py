import cv2
import RPi.GPIO as GPIO
import time
import os

# Setup for Ultrasonic Sensor
GPIO.setmode(GPIO.BCM)
TRIG = 23  # GPIO pin for Trigger
ECHO = 24  # GPIO pin for Echo

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Function to check Ultrasonic Sensor
def check_ultrasonic_sensor():
    try:
        # Send a pulse to trigger the ultrasonic sensor
        GPIO.output(TRIG, GPIO.LOW)  # Ensure Trigger is low
        time.sleep(0.5)  # Give the sensor time to settle
        GPIO.output(TRIG, GPIO.HIGH)  # Send trigger pulse
        time.sleep(0.00001)  # Wait for 10 microseconds
        GPIO.output(TRIG, GPIO.LOW)  # Stop the trigger pulse

        # Measure the time it takes for the Echo to return
        while GPIO.input(ECHO) == GPIO.LOW:
            pulse_start = time.time()  # Record start time
        
        while GPIO.input(ECHO) == GPIO.HIGH:
            pulse_end = time.time()  # Record end time

        # Calculate the distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Distance = (Time * Speed of sound in cm/s)
        distance = round(distance, 2)

        # Check if the sensor is working properly
        if distance <= 40 and distance > 0:
            return f"Ultrasonic Sensor: OK, Distance = {distance} cm"
        else:
            return "INF"
    except Exception as e:
        return f"Ultrasonic Sensor: Error, {str(e)}"
    finally:
        GPIO.cleanup()  # Clean up the GPIO setup

# Function to check Webcam (Camera) connection
def monitor_webcam():
    try:
        cap = cv2.VideoCapture(0)  # Attempt to open the first video device
        if cap.isOpened():
            ret, frame = cap.read()  # Attempt to grab a frame
            if ret:
                cap.release()
                return "Webcam: OK, Camera detected"
            else:
                cap.release()
                return "Webcam: Error, Camera not responding"
        else:
            return "Webcam: Error, Camera not detected"
    except Exception as e:
        return f"Webcam: Error, {str(e)}"

# Main function to check both components
def check_components():
    # Check the camera status
    camera_status = monitor_webcam()
    
    # Check the ultrasonic sensor status
    ultrasonic_status = check_ultrasonic_sensor()

    # Print results
    print(camera_status)
    print(ultrasonic_status)

if __name__ == "__main__":
    check_components()
