import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ActuatorSimNode(Node):
    """
    Simulates the Servo Driver.
    In reality, this listens to commands and writes PWM to the servo.
    """
    def __init__(self):
        super().__init__('actuator_sim')
        self.sub = self.create_subscription(
            Float32, 'actuators/camber_cmd', self.listener_callback, 10)
        self.get_logger().info("Actuator Interface Started")

    def listener_callback(self, msg):
        target_angle = msg.data
        
        # HARDWARE CONTROL HERE
        # e.g., servo.write(target_angle)
        
        self.get_logger().info(f"Writing Servo Angle: {target_angle:.2f} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()