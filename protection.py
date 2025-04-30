import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus2
import time

# I2C Address for ADS1115
ADS1115_ADDRESS = 0x48  
ADC_CHANNEL = 0  # Using A0

# Voltage divider values
R1 = 100000.0  # 100kΩ
R2 = 10000.0   # 10kΩ
DIVIDER_RATIO = (R1 + R2) / R2  # Scale factor

# ADS1115 Registers
CONFIG_REG = 0x01
CONVERSION_REG = 0x00
CONFIG_VALUE = 0x8483  # Single-ended mode, A0, 4.096V range

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(Float32, '/battery_voltage', 10)
        self.timer = self.create_timer(1.0, self.read_battery_voltage)
        self.i2c = smbus2.SMBus(1)
        self.configure_adc()
        self.get_logger().info('Battery Monitor Node Started')

    def configure_adc(self):
        config_bytes = [CONFIG_VALUE >> 8, CONFIG_VALUE & 0xFF]
        self.i2c.write_i2c_block_data(ADS1115_ADDRESS, CONFIG_REG, config_bytes)

    def read_adc(self):
        data = self.i2c.read_i2c_block_data(ADS1115_ADDRESS, CONVERSION_REG, 2)
        raw_adc = (data[0] << 8) | data[1]
        if raw_adc > 32767:
            raw_adc -= 65536  # Convert to signed 16-bit
        return raw_adc

    def read_battery_voltage(self):
        raw_adc = self.read_adc()
        voltage = (raw_adc * 4.096) / 32768.0  # Convert ADC value to voltage
        battery_voltage = voltage * DIVIDER_RATIO  # Convert back to real battery voltage

        msg = Float32()
        msg.data = battery_voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery Voltage: {battery_voltage:.2f}V')

        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

