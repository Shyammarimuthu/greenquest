import RPi.GPIO as GPIO
import time
from datetime import datetime

# Constants
ROVER_ID = "Rover_1"
RPI_NO   = "Rpi_1"

def monitor_buzzer(pin, comp_name="buzzer"):
    """
    Drive the buzzer pin HIGH for 2 seconds, then LOW,
    and print status as:
      comp_name:Rover_id,Rpi_no,comp_status,date,time
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    
    try:
        # Sound the buzzer
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(pin, GPIO.LOW)
        status = f"{comp_name} detected"
    except Exception:
        status = f"{comp_name} not detected"
    finally:
        GPIO.cleanup()

    # Timestamp
    now      = datetime.now()
    date_str = now.strftime("%d/%m/%Y")
    time_str = now.strftime("%I:%M%p").lower()

    # Print in your format
    print(f"{comp_name}:{ROVER_ID},{RPI_NO},{status},{date_str},{time_str}")

# Example usage:
if __name__ == "__main__":
    BUZZER_PIN = 26
    monitor_buzzer(BUZZER_PIN, "buzzer")
