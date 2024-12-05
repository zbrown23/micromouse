import numpy as np
from controls import Pose2d


class OffCenterController:
    def __init__(self, sim, kp=0.5, wheel_dia=0.032, track_width=0.05, ctrl_pt_dist=0.01):
        self.sim = sim
        self.kp = kp
        self.wheel_rad = wheel_dia / 2
        self.track_width = track_width
        self.ctrl_pt_dist = ctrl_pt_dist
        self.is_driving = False

    def update(self, robot_pos: Pose2d, target_pos: Pose2d):
        ref = np.array([target_pos.x, target_pos.y])
        error = ref - self.compute_off_center_pt(robot_pos)
        u = self.kp * self.compute_j_inv(robot_pos.theta) * error
        return u[0], u[1]

    def compute_j_inv(self, theta):
        j_inv = (1 / self.wheel_rad) * np.array([
            [np.cos(theta) - (self.track_width / (2 * self.ctrl_pt_dist)) * np.sin(theta),
             np.sin(theta) + (self.track_width / (2 * self.ctrl_pt_dist)) * np.cos(theta)],
            [np.cos(theta) + (self.track_width / (2 * self.ctrl_pt_dist)) * np.sin(theta),
             np.sin(theta) - (self.track_width / (2 * self.ctrl_pt_dist)) * np.cos(theta)]
        ])
        return j_inv

    def compute_off_center_pt(self, pose: Pose2d):
        off_center_pt = np.array([
            [pose.x + self.a * np.cos(pose.theta)],
            [pose.y + self.a * np.sin(pose.theta)]
        ])
        return off_center_pt
