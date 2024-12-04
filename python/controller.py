import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
from util import Pose2d
from controls import LQR


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


class Robot:
    def __init__(self, sim, controller: LTVUnicycleController, wheel_dia=0.032, track_width=0.05):
        self.track_width = track_width
        self.wheel_dia = wheel_dia
        self.controller = controller
        if sim is None:
            raise Exception("Need a ZMQ client to drive!")
        self.sim = sim
        self.l_wheel = self.sim.getObject('/Micromouse/WheelL_Joint')
        self.r_wheel = self.sim.getObject('/Micromouse/WheelR_Joint')
        self.center = self.sim.getObject('/Micromouse/Center')

    def drive(self, v, omega):
        omega_r = (v + omega * (self.track_width / 2)) / self.wheel_dia
        omega_l = (v - omega * (self.track_width / 2)) / self.wheel_dia
        self.sim.setJointTargetVelocity(self.r_wheel, -omega_r)
        self.sim.setJointTargetVelocity(self.l_wheel, omega_l)

    def driveToPoint(self, pointHandle, error_threshold):
        point_pos = Pose2d.from_sim(self.sim.getObjectPose(pointHandle, self.sim.handle_world))
        robot_pos = Pose2d.from_sim(self.sim.getObjectPose(self.center, self.sim.handle_world))
        error = point_pos.relativeTo(robot_pos).magnitude()
        print(robot_pos)
        if error > error_threshold:
            out = self.controller.calculate(robot_pos, point_pos, 0.1, 0.5)
            print(out)
            self.drive(out[0], -out[1])
            return False
        else:
            self.drive(0, 0)
            return True


