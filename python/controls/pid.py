import numpy as np


class PIDF:
    def __init__(
        self,
        k_p: float = 0.0,
        k_i: float = 0.0,
        k_d: float = 0.0,
        k_f: float = 0.0,
        sample_time: float = 0.1,
        tau: float = None,
        out_min: float = None,
        out_max: float = None,
        int_min: float = None,
        int_max: float = None,
    ):
        """
        Initialize the PIDF controller.
        :param k_p: proportional gain
        :param k_i: integral gain
        :param k_d: derivative gain
        :param k_f: feedforward gain
        :param sample_time: time step (seconds)
        :param tau: low-pass filter time constant for the derivative term
        :param out_min: minimum output value
        :param out_max: maximum output value
        :param int_min: minimum integrator value
        :param int_max: maximum integrator value
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
        Update the PIDF controller based on the setpoint and measurement.
        :param setpoint: desired target value
        :param measurement: current process value
        :return: control output
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
        if self.tau is not None:
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

    def set_gains(self, k_p: float, k_i: float, k_d: float, k_f: float):
        """
        Update the PIDF gains.
        :param k_p: proportional gain
        :param k_i: integral gain
        :param k_d: derivative gain
        :param k_f: feedforward gain
        """
        self.k_p, self.k_i, self.k_d, self.k_f = k_p, k_i, k_d, k_f


# Example Usage
if __name__ == "__main__":
    pidf = PIDF(
        k_p=1.0, k_i=0.1, k_d=0.01, k_f=0.5,
        sample_time=0.1, tau=0.5,
        out_min=-10, out_max=10,
        int_min=-5, int_max=5
    )

    setpoint = 10.0
    measurement = 0.0

    for _ in range(50):
        output = pidf.update(setpoint, measurement)
        measurement += output * 0.1  # Simulated system response
        print(f"Setpoint: {setpoint}, Measurement: {measurement:.2f}, Output: {output:.2f}")
