import numpy as np
from controls import Pose2d


class Odometry:
    def __init__(self, wheel_dia=0.032, track_width=0.05):
        pass

    def update(self, omega_l, omega_r, dt) -> Pose2d:
        """
        update the internal pose with the current wheel speed measurements
        :param omega_l: left wheel angular velocity
        :param omega_r: right wheel angular velocity
        :param dt: time since last update
        :return: the new pose.
        """
        pass

    def reset(self, pose: Pose2d):
        """
        Resets the odometry to a specified pose.
        :param pose: the pose to reset to.
        :return:
        """
        pass
