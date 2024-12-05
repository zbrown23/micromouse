import numpy as np
from typing import Optional


class PIDF:
    def __init__(
            self,
            k_p: float = 0.0,
            k_i: float = 0.0,
            k_d: float = 0.0,
            k_f: float = 0.0,
            sample_time: float = 0.1,
            tau: Optional[float] = None,
            out_min: Optional[float] = None,
            out_max: Optional[float] = None,
            int_min: Optional[float] = None,
            int_max: Optional[float] = None,
    ):
        """
        PIDF Controller.

        Args:
        - k_p, k_i, k_d, k_f: Gains for proportional, integral, derivative, and feedforward terms.
        - sample_time: Time step (seconds).
        - tau: Time constant for low-pass filtering of the derivative term (optional).
        - out_min, out_max: Minimum and maximum limits for the controller output (optional).
        - int_min, int_max: Minimum and maximum limits for the integrator term (optional).
        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.k_f = k_f
        self.sample_time = sample_time
        self.tau = tau
        self.out_min = out_min
        self.out_max = out_max
        self.int_min = int_min
        self.int_max = int_max

        # Initialize state
        self.integrator = 0.0
        self.prev_error = 0.0
        self.differentiator = 0.0
        self.prev_measurement = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """
        Compute the control output based on the setpoint and measurement.

        Args:
        - setpoint: Desired target value.
        - measurement: Current process value.

        Returns:
        - Control output.
        """
        error = setpoint - measurement

        # Proportional term
        proportional = self.k_p * error

        # Integral term with clamping
        self.integrator += self.k_i * self.sample_time * (error + self.prev_error)
        self.integrator = np.clip(
            self.integrator,
            self.int_min if self.int_min is not None else -np.inf,
            self.int_max if self.int_max is not None else np.inf,
        )

        # Derivative term with optional low-pass filtering
        if self.tau:
            self.differentiator = (
                    -(2 * self.k_d * (measurement - self.prev_measurement)
                      + (2 * self.tau - self.sample_time) * self.differentiator)
                    / (2 * self.tau + self.sample_time)
            )
        else:
            self.differentiator = self.k_d * (measurement - self.prev_measurement) / self.sample_time

        # Feedforward term
        feedforward = self.k_f * setpoint

        # Compute the output and apply clamping
        output = proportional + self.integrator + self.differentiator + feedforward
        output = np.clip(
            output,
            self.out_min if self.out_min is not None else -np.inf,
            self.out_max if self.out_max is not None else np.inf,
        )

        # Update state for the next iteration
        self.prev_error = error
        self.prev_measurement = measurement

        return output
