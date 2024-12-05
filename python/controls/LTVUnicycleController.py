import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
import Pose2d
from lqr import LQR


class LTVUnicycleController:
    """
    A linear time-varying unicycle controller.
    """

    def __init__(self, q, r, dt):
        self.q = q
        self.r = r
        self.dt = dt

    def calculate(self, pose: Pose2d, desired_pose: Pose2d, desired_v, desired_omega):
        """
        Steps the output of the unicycle controller.
        :param pose: the current pose of the robot
        :param desired_pose: the desired pose of the robot
        :param desired_v: the desired linear velocity of the robot
        :param desired_omega: the desired angular velocity of the robot center
        :return: the output angular and linear velocity of the robot
        """
        error = desired_pose.relativeTo(pose)
        e = np.array([[error.x], [error.y], [error.theta]])
        A = np.zeros((3, 3))
        if abs(desired_v) < 1e-9:  # this linearization strategy does not being still, the gains will blow up to +- inf
            v_desired = 1e-9
        A[1, 2] = desired_v  # set the velocity
        B = np.array([[1, 0], [0, 0], [0, 1]])
        K = LQR(A, B, self.q, self.r, self.dt).K  # get the K matrix of the full-state-feedback control law
        u = K @ e  # multiply the error vector by the K matrix to get the control input
        return desired_v + u[0, 0], desired_omega + u[1, 0]
