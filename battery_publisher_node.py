import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header

# ROS2 Publisher Node
class BatteryPublisherNode(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_state)  # Publish every 1 second
        self.battery_state = BatteryState()

        # Battery parameters
        self.full_voltage = 42.0  # Full voltage (100% battery)
        self.voltage_at_20 = 38.0  # 20% at 38V
        self.min_voltage = 37.0  # Minimum voltage for 0% battery

        # Voltage divider multiplying factor
        self.divider_factor = 11.2

        # I2C initialization flags
        self.ads = None
        self.chan = None
        self.i2c_initialized = False

    def initialize_i2c(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            if not i2c.try_lock():
                return False
            i2c.unlock()

            self.ads = ADS.ADS1115(i2c)
            self.chan = AnalogIn(self.ads, ADS.P0)  # A0 as the input channel
            self.i2c_initialized = True
            self.get_logger().info("I2C and ADS1115 successfully initialized.")
            return True
        except Exception as e:
            self.get_logger().warn(f"I2C not available yet: {e}")
            return False

    def publish_battery_state(self):
        if not self.i2c_initialized:
            initialized = self.initialize_i2c()
            if not initialized:
                self.get_logger().info("Waiting for I2C device...")
                return  # Skip publishing until I2C is ready

        try:
            # Get the measured voltage
            measured_voltage = self.chan.voltage
            actual_voltage = measured_voltage * self.divider_factor  # Apply multiplying factor

            # Calculate battery percentage
            battery_percentage = self.calculate_battery_percentage(actual_voltage)

            # Round values to 1 decimal place
            actual_voltage = round(actual_voltage, 1)
            battery_percentage = round(battery_percentage, 1)

            # Set up the battery state message
            self.battery_state.header = Header()
            self.battery_state.header.stamp = self.get_clock().now().to_msg()
            self.battery_state.voltage = actual_voltage
            self.battery_state.percentage = battery_percentage
            self.battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            self.battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

            # Publish the message
            self.publisher.publish(self.battery_state)
            self.get_logger().info(f'Publishing BatteryState: {actual_voltage}V, {battery_percentage}%')

        except Exception as e:
            self.get_logger().error(f"Error reading battery voltage: {e}")
            self.i2c_initialized = False  # Retry I2C setup in next cycle

    def calculate_battery_percentage(self, voltage):
        # Ensure voltage is within the range of min_voltage and full_voltage
        if voltage >= self.full_voltage:
            return 100.0
        elif voltage <= self.min_voltage:
            return 0.0
        elif voltage >= self.voltage_at_20:
            # Linear interpolation for voltage between 38V (20%) and 42V (100%)
            return 20 + ((voltage - self.voltage_at_20) / (self.full_voltage - self.voltage_at_20)) * 80
        else:
            # Linear interpolation for voltage between 37V (0%) and 38V (20%)
            return ((voltage - self.min_voltage) / (self.voltage_at_20 - self.min_voltage)) * 20

def main(args=None):
    rclpy.init(args=args)
    battery_publisher_node = BatteryPublisherNode()
    rclpy.spin(battery_publisher_node)
    battery_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
