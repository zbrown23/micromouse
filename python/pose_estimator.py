import math
import numpy as np
from controls import Pose2d


class PoseEstimator:
    def __init__(self, sim, initial_pose: Pose2d = None, alpha: float = 0.98, beta: float = 0.005):
        self.sim = sim
        self.gyro = Gyro(sim)
        self.odometry = Odometry(initial_pose)
        self.filter = AlphaBetaFilter2D(alpha, beta)
        self.l_wheel = self.sim.getObject('/micromouse/l_wheel_joint')
        self.r_wheel = self.sim.getObject('/micromouse/r_wheel_joint')

    def update(self, dt):
        omega_l = self.sim.getJointVelocity(self.l_wheel)
        omega_r = self.sim.getJointVelocity(self.r_wheel)
        odom_pose = self.odometry.update(omega_l, omega_r, dt)
        gyro_rate = self.gyro.read()
        return self.filter.update(odom_pose, gyro_rate, dt)


class Gyro:
    def __init__(self, sim):
        """
        Simulates a gyro by reading the angular velocity of the robot from the simulation.
        :param sim: the simulation handle
        """
        self.sim = sim
        if sim is None:
            raise Exception("Need a sim handle!")
        self.robot = sim.getObject('/micromouse')

    def read(self) -> float:
        _, angular_vel = self.sim.getVelocity(self.robot)
        angular_vel = np.array(angular_vel)
        angular_vel = np.linalg.norm(angular_vel)
        angular_vel += np.random.normal(0, 0.025)  # add noise similar to the noise in a MEMS gyro
        return angular_vel


class Odometry:
    def __init__(self, initial_pose: Pose2d = None, wheel_dia=0.032, track_width=0.05):
        self.wheel_radius = wheel_dia / 2
        self.track_width = track_width
        if initial_pose is None:
            self.pose = Pose2d()
        else:
            self.pose = initial_pose

    def update(self, omega_l, omega_r, dt) -> Pose2d:
        """
        update the internal pose with the current wheel speed measurements
        :param omega_l: left wheel angular velocity
        :param omega_r: right wheel angular velocity
        :param dt: time since last update
        :return: the new pose.
        """
        v_l = omega_l * self.wheel_radius
        v_r = -omega_r * self.wheel_radius
        v = (v_l + v_r) / 2
        w = (v_r - v_l) / self.track_width

        dy = v * math.cos(self.pose.theta) * dt
        dx = v * math.sin(self.pose.theta) * dt
        d_theta = w * dt

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta = wrap_angle(self.pose.theta + d_theta)

        return self.pose

    def reset(self, pose: Pose2d):
        """
        Resets the odometry to a specified pose.
        :param pose: the pose to reset to.
        :return:
        """
        self.pose = pose


class AlphaBetaFilter2D:
    def __init__(self, alpha: float, beta: float):
        self.alpha = alpha  # Position weight
        self.beta = beta  # Velocity weight

        # Initial pose and velocities
        self.pose = Pose2d()  # (x, y, theta)
        self.vx = 0.0  # Velocity in x
        self.vy = 0.0  # Velocity in y
        self.omega = 0.0  # Angular velocity

    def update(self, odom_pose: Pose2d, gyro_rate: float, dt: float) -> Pose2d:
        # Integrate gyro rate to update orientation
        self.pose.theta += gyro_rate * dt

        # Predict step
        self.pose.x += self.vx * dt
        self.pose.y += self.vy * dt

        # Correct step using odometry
        self.pose.x += self.alpha * (odom_pose.x - self.pose.x)
        self.pose.y += self.alpha * (odom_pose.y - self.pose.y)
        self.pose.theta += self.alpha * (odom_pose.theta - self.pose.theta)

        # Update velocities
        self.vx += self.beta * ((odom_pose.x - self.pose.x) / (dt + 0.00001))
        self.vy += self.beta * ((odom_pose.y - self.pose.y) / (dt + 0.00001))
        self.omega = gyro_rate  # Use the current gyro rate

        # Normalize theta to [-pi, pi]
        self.pose.theta = wrap_angle(self.pose.theta)

        return self.pose


def wrap_angle(angle):
    """
    Wraps an angle in radians to the range [-π, π].
    :param angle: The angle in radians.
    :return: The wrapped angle.
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
