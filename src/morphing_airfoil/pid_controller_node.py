import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import statistics
from .pid import PIDController

class PIDAirfoilController(Node):
    def __init__(self):
        super().__init__('pid_airfoil_controller')

        # Declare parameters
        self.declare_parameter('kp', 0.1)  # Proportional gain
        self.declare_parameter('ki', 0.05)   # Integral gain
        self.declare_parameter('kd', 0.05)  # Derivative gain
        self.declare_parameter('target_lift', 5.0)  # Target lift in Newtons
        self.declare_parameter('calibration_duration', 5.0) # Seconds to wait/calibrate

        # Get params
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.target_lift = self.get_parameter('target_lift').value
        self.calibration_duration = self.get_parameter('calibration_duration').value
        
        # Initialize PID Controller
        # Servo limits: 0.0 to 180.0 degrees
        self.pid = PIDController(kp, ki, kd, min_out=0.0, max_out=180.0)

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
        self.last_lift_time = self.get_clock().now()
        self.get_logger().info(f"PID Controller Started. Kp={kp}, Ki={ki}, Kd={kd}, Target Lift={self.target_lift}N. Calibrating for {self.calibration_duration}s...")

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

        # --- PID CONTROL LOGIC ---
        
        # Calculate time delta for PID
        dt = (current_time - self.last_lift_time).nanoseconds / 1e9
        
        # Rate Limiting: Only publish every 0.1s (10Hz)
        if (current_time - self.last_pub_time).nanoseconds / 1e9 < 0.1:
            return
        
        # Safeguard against invalid dt
        if dt <= 0.0:
            return

        # 1. Subtract Baseline (Auto-Tare) to get adjusted lift
        adjusted_lift = current_lift - self.baseline_lift
        
        # 2. Calculate target (baseline + desired lift)
        target_adjusted = self.target_lift
        
        # 3. Use PID to compute servo angle
        # PID error = target - current (we want to increase lift when below target)
        camber_cmd = self.pid.update(target_adjusted, adjusted_lift, dt)

        self.last_pub_time = current_time
        self.last_lift_time = current_time

        # Publish command
        cmd_msg = Float32()
        cmd_msg.data = camber_cmd
        self.camber_pub.publish(cmd_msg)

        # Log for debugging
        self.get_logger().info(f"Raw: {current_lift:.2f}N | Adj: {adjusted_lift:.2f}N | Target: {target_adjusted:.2f}N | Cmd: {camber_cmd:.1f}° | dt: {dt:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    node = PIDAirfoilController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

