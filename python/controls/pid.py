class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None), i_term_limits=(None, None)):
        """
        Initialize the PID controller.

        Parameters:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limits: Tuple (min_output, max_output), limits for PID output
        i_term_limits: Tuple (min_i_term, max_i_term), limits for integral term
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.i_term_limits = i_term_limits

        self.last_error = 0
        self.integral = 0

    def reset(self):
        """Reset the controller."""
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        """
        Update the PID controller.

        Parameters:
        error: The current error (setpoint - measured_value)
        dt: Time step in seconds (must be > 0)

        Returns:
        Control output
        """
        if dt <= 0:
            raise ValueError("dt must be greater than zero.")

        # Proportional term
        p_term = self.kp * error

        # Integral term with limits
        self.integral += error * dt
        if self.i_term_limits[0] is not None:
            self.integral = max(self.i_term_limits[0], self.integral)
        if self.i_term_limits[1] is not None:
            self.integral = min(self.i_term_limits[1], self.integral)
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        # Combine terms
        output = p_term + i_term + d_term

        # Apply output limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        # Update last_error
        self.last_error = error

        return output
