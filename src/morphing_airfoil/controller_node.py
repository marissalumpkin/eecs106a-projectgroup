import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import statistics

class AirfoilController(Node):
    def __init__(self):
        super().__init__('airfoil_controller')

        # Declare parameters
        self.declare_parameter('reactive_gain', 1.0)
        self.declare_parameter('calibration_duration', 5.0) # Seconds to wait/calibrate

        # Get params
        self.reactive_gain = self.get_parameter('reactive_gain').value
        self.calibration_duration = self.get_parameter('calibration_duration').value

        # Calibration State
        self.is_calibrated = False
        self.calibration_samples = []
        self.baseline_lift = 0.0
        self.start_time = self.get_clock().now()

        # Publishers & Subscribers
        self.lift_sub = self.create_subscription(
            Float32, 'sensors/lift', self.lift_callback, 10)
        self.camber_pub = self.create_publisher(Float32, 'actuators/camber_cmd', 10)

        self.last_pub_time = self.get_clock().now()
        self.get_logger().info(f"Reactive Controller Started. Gain: {self.reactive_gain}. Calibrating for {self.calibration_duration}s...")

    def lift_callback(self, msg):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        current_lift = msg.data

        # --- CALIBRATION PHASE ---
        if not self.is_calibrated:
            if elapsed_time < self.calibration_duration:
                # Collect samples
                self.calibration_samples.append(current_lift)
                if len(self.calibration_samples) % 10 == 0:
                    self.get_logger().info(f"Calibrating... {elapsed_time:.1f}/{self.calibration_duration}s (Sample: {current_lift:.2f})")
                return # Do not move servo yet
            else:
                # Finish Calibration
                if len(self.calibration_samples) > 0:
                    self.baseline_lift = statistics.median(self.calibration_samples)
                    self.is_calibrated = True
                    self.get_logger().info(f"CALIBRATION COMPLETE. Baseline set to: {self.baseline_lift:.2f}")
                else:
                    self.get_logger().warn("Calibration failed (no samples). Retrying...")
                    self.start_time = current_time # Reset timer
                    return

        # --- REACTIVE CONTROL LOGIC ---
        
        # Rate Limiting: Only publish every 0.1s (10Hz)
        if (current_time - self.last_pub_time).nanoseconds / 1e9 < 0.1:
            return

        # 1. Subtract Baseline (Auto-Tare)
        adjusted_lift = current_lift - self.baseline_lift
        
        # 2. Angle = |Force| * Gain
        camber_cmd = abs(adjusted_lift) * self.reactive_gain

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
        self.get_logger().info(f"Raw: {current_lift:.0f} | Adj: {adjusted_lift:.0f} | Cmd: {camber_cmd:.0f}")

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