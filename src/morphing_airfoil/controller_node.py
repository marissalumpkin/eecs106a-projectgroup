import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class AirfoilController(Node):
    def __init__(self):
        super().__init__('airfoil_controller')

        # Declare parameters
        # reactive_gain: How many degrees to curve per unit of force (grams)
        # Example: Gain=0.5, Force=100g -> Angle=50 deg
        self.declare_parameter('reactive_gain', 1.0)

        # Get params
        self.reactive_gain = self.get_parameter('reactive_gain').value

        # Publishers & Subscribers
        self.lift_sub = self.create_subscription(
            Float32, 'sensors/lift', self.lift_callback, 10)
        self.camber_pub = self.create_publisher(Float32, 'actuators/camber_cmd', 10)

        self.last_time = self.get_clock().now()
        self.last_pub_time = self.get_clock().now()
        self.get_logger().info(f"Reactive Controller Started. Gain: {self.reactive_gain}")

    def lift_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Rate Limiting: Only publish every 0.1s (10Hz) to avoid clogging Serial
        if (current_time - self.last_pub_time).nanoseconds / 1e9 < 0.1:
            return

        current_lift = msg.data
        
        # --- REACTIVE CONTROL LOGIC ---
        # Angle = Force * Gain
        camber_cmd = current_lift * self.reactive_gain

        # Clamp to Safe Limits (0 to 180 degrees)
        if camber_cmd < 0.0:
            camber_cmd = 0.0
        elif camber_cmd > 180.0:
            camber_cmd = 180.0

        self.last_pub_time = current_time

        # Publish command
        cmd_msg = Float32()
        cmd_msg.data = camber_cmd
        self.camber_pub.publish(cmd_msg)

        # Log for debugging
        self.get_logger().info(f"Lift (Force): {current_lift:.2f} | Cmd (Angle): {camber_cmd:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AirfoilController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()