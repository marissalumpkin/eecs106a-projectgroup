import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math

class SensorSimNode(Node):
    """
    Simulates the HX711 Load Cell and Pitot Tube.
    Replace logic in 'timer_callback' with real hardware calls later.
    """
    def __init__(self):
        super().__init__('sensor_sim')
        self.lift_pub = self.create_publisher(Float32, 'sensors/lift', 10)
        self.airspeed_pub = self.create_publisher(Float32, 'sensors/airspeed', 10)

        # Run at 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info("Sensor Simulation Started")

    def timer_callback(self):
        # 1. READ HARDWARE (Simulated here)
        # In real implementation:
        # lift_raw = hx711.read()
        # airspeed_raw = adc.read()
        
        # Simulating a system where lift oscillates naturally
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        sim_lift = 5.0 + 2.0 * math.sin(t) + random.uniform(-0.1, 0.1) # Base + Sine wave + Noise
        sim_airspeed = 15.0 + random.uniform(-0.5, 0.5)

        # 2. PUBLISH
        lift_msg = Float32()
        lift_msg.data = sim_lift
        self.lift_pub.publish(lift_msg)

        speed_msg = Float32()
        speed_msg.data = sim_airspeed
        self.airspeed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()