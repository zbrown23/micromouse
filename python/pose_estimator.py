import math
import numpy as np
from controls import Pose2d


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

    def read(self):
        _, angular_vel = self.sim.getVelocity(self.robot)
        angular_vel = np.array(angular_vel)
        angular_vel = np.linalg.norm(angular_vel)
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
