import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .pid import PIDController
import time

class AirfoilController(Node):
    def __init__(self):
        super().__init__('airfoil_controller')

        # Declare parameters (Tuning these is a key project task!)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('target_lift', 5.0) # Target lift in Newtons (example)

        # Get params
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.target_lift = self.get_parameter('target_lift').value

        # Initialize PID
        # Servo limits: Assume 0.0 is flat, 90.0 is max curve
        self.pid = PIDController(kp, ki, kd, min_out=0.0, max_out=90.0)

        # Publishers & Subscribers
        self.lift_sub = self.create_subscription(
            Float32, 'sensors/lift', self.lift_callback, 10)
        self.camber_pub = self.create_publisher(Float32, 'actuators/camber_cmd', 10)

        self.last_time = self.get_clock().now()
        self.get_logger().info(f"Controller Started. Target Lift: {self.target_lift}")

    def lift_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Simple safeguard against jumps or first run
        if dt <= 0.0:
            return

        current_lift = msg.data
        
        # Compute control signal (Camber Angle)
        camber_cmd = self.pid.update(self.target_lift, current_lift, dt)

        # Publish command
        cmd_msg = Float32()
        cmd_msg.data = camber_cmd
        self.camber_pub.publish(cmd_msg)

        # Log for debugging (Task 3a in proposal)
        self.get_logger().debug(f"Lift: {current_lift:.2f} | Cmd: {camber_cmd:.2f} | dt: {dt:.4f}")

        self.last_time = current_time

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