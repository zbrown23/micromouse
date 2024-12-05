import math

import numpy as np
from controls import Pose2d


class Odometry:
    def __init__(self, wheel_dia=0.032, track_width=0.05):
        self.wheel_radius = wheel_dia / 2
        self.track_width = track_width
        self.pose = Pose2d()

    def update(self, omega_l, omega_r, dt) -> Pose2d:
        """
        update the internal pose with the current wheel speed measurements
        :param omega_l: left wheel angular velocity
        :param omega_r: right wheel angular velocity
        :param dt: time since last update
        :return: the new pose.
        """
        v_l = omega_l * self.wheel_radius
        v_r = omega_r * self.wheel_radius
        v = (v_l + v_r) / 2
        w = (v_r - v_l) / self.track_width

        dx = v * math.cos(self.pose.theta) * dt
        dy = v * math.sin(self.pose.theta) * dt
        dtheta = w * dt

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta = self.wrap_angle(self.pose.theta + dtheta)

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