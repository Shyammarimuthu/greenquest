import board
import busio
import time
from datetime import datetime

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Constants
ROVER_ID = "Rover_1"
RPI_NO   = "Rpi_1"
# Divider values
R1 = 100_000  # top resistor (Ω)
R2 =  10_000  # bottom resistor (Ω)
DIV_RATIO = 11.14 # = 11

def monitor_voltage_sensor(comp_name="voltage_sensor"):
    """
    Reads the divided voltage from ADS1115 A0,
    scales it back up for the actual battery voltage,
    and prints in:
      comp_name:Rover_id,Rpi_no,comp_status,date,time
    """
    try:
        # I2C & ADS1115 init
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        chan = AnalogIn(ads, ADS.P0)

        time.sleep(0.1)  # allow ADC to settle
        measured = chan.voltage          # voltage at A0 (divided)
        actual   = round(measured * DIV_RATIO, 2)  # battery voltage

        comp_status = f"{comp_name} detected {actual}V"

    except Exception:
        comp_status = f"{comp_name} not detected"

    # Timestamp
    now      = datetime.now()
    date_str = now.strftime("%d/%m/%Y")
    time_str = now.strftime("%I:%M%p").lower()

    print(f"{comp_name}:{ROVER_ID},{RPI_NO},{comp_status},{date_str},{time_str}")

if __name__ == "__main__":
    monitor_voltage_sensor()

