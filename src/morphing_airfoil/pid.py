class PIDController:
    def __init__(self, kp, ki, kd, min_out, max_out, dt=None):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param min_out: Minimum output (e.g., servo angle limit)
        :param max_out: Maximum output
        :param dt: Time step in seconds (optional, can be calculated dynamically)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.fixed_dt = dt

    def update(self, setpoint, measurement, dt_actual=None):
        """
        Computes the control signal.
        :param setpoint: Desired Lift
        :param measurement: Current Lift
        :param dt_actual: Time elapsed since last update
        """
        error = setpoint - measurement
        dt = self.fixed_dt if self.fixed_dt else dt_actual

        if dt is None or dt <= 0.0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Compute output
        output = p_term + i_term + d_term

        # Clamp output to physical limits (e.g., servo range)
        output = max(min(output, self.max_out), self.min_out)

        self.prev_error = error

        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0