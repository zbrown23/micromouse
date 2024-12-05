import numpy as np
from controls import Pose2d


class OffCenterController:
    def __init__(self, sim, kp, kd, control_pt_handle, wheel_radius, track_width, pose2d):
        self.is_driving = False
        self.control_pt_handle = control_pt_handle
        self.r = wheel_radius
        self.w = track_width
        self.pose2d = pose2d
        self.a = 0.01

        self.kp = kp
        self.kd = kd

        self.sim = sim
        self.control_pt_pos = self.sim.getObjectPosition(control_pt_handle, sim.handle_world)
        self.control_pt_angle = self.sim.getObjectOrientation(control_pt_handle, sim.handle_world)




        pass

    def drive_to_point(self):
        # give the robot a new point to drive to.
        # return false if the robot isn't at the point, and true when it reaches.
        pass

    def update(self, robot_pos):
        if self.is_driving:

            pass
        omega_l = 0.0
        omega_r = 0.0
        self.off_center_pt = compute_off_center_pt(pose2d)
        error = np.array([self.pose2d[0] - self.off_center_pt[0], self.pose2d[1] - self.off_center_pt[1]])
        jInv = compute_jinv(self, self.pose2d[3])
        [omega_l, omega_r] = jInv * error * self.kp
        return omega_l, omega_r


def compute_jinv(self, theta):
    jinv = (1 /self.r) * np.array([
        [np.cos(theta) - (self.w / (2 * self.a)) * np.sin(theta), np.sin(theta) + (self.w / (2 * self.a)) * np.cos(theta)],
        [np.cos(theta) + (self.w / (2 * self.a)) * np.sin(theta), np.sin(theta) - (self.w / (2 * self.a)) * np.cos(theta)]
    ])
    return jinv
    pass


def compute_off_center_pt(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta

    offcenterpt = np.array([
        [x + self.a * np.cos(theta)],
        [y + self.a * np.sin(theta)]
    ])

    return offcenterpt

