import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math
import time
import serial

class SensorSimNode(Node):
    """
    Reads from Arduino via Serial (Lift & Airspeed).
    If 'use_sim' is True, generates fake data.
    """
    def __init__(self):
        super().__init__('sensor_driver')
        
        # Parameters
        self.declare_parameter('use_sim', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.use_sim = self.get_parameter('use_sim').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        # Publishers
        self.lift_pub = self.create_publisher(Float32, 'sensors/lift', 10)
        self.airspeed_pub = self.create_publisher(Float32, 'sensors/airspeed', 10)

        self.ser = None

        # Initialize Hardware or Sim
        if not self.use_sim:
            self.setup_serial()
        
        if self.use_sim:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Sensor Node Started in SIMULATION MODE")
        else:
            self.get_logger().info(f"Sensor Node Started in HARDWARE MODE (Serial: {self.serial_port})")

        # Run at 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

    def setup_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Serial: {e}")
            self.get_logger().warn("Reverting to SIMULATION MODE due to connection failure.")
            self.use_sim = True

    def timer_callback(self):
        current_lift = 0.0
        current_airspeed = 0.0

        if self.use_sim:
            # --- SIMULATION LOGIC ---
            t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
            current_lift = 5.0 + 2.0 * math.sin(t) + random.uniform(-0.1, 0.1)
            current_airspeed = 15.0 + random.uniform(-0.5, 0.5)
        else:
            # --- HARDWARE LOGIC (SERIAL) ---
            if self.ser and self.ser.is_open:
                try:
                    # Read line: "L:5.23,A:12.50"
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8').strip()
                        # Simple parsing
                        parts = line.split(',')
                        for part in parts:
                            if part.startswith('L:'):
                                current_lift = float(part.split(':')[1])
                            elif part.startswith('A:'):
                                current_airspeed = float(part.split(':')[1])
                except Exception as e:
                    self.get_logger().warn(f"Serial Read Error: {e}")

        # Publish
        lift_msg = Float32()
        lift_msg.data = float(current_lift)
        self.lift_pub.publish(lift_msg)

        speed_msg = Float32()
        speed_msg.data = float(current_airspeed)
        self.airspeed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()