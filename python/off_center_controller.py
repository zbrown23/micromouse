import numpy as np
from controls import Pose2d


class OffCenterController:
    def __init__(self, kp, kd, control_pt_handle, track_width, ):
        self.is_driving = False
        pass

    def drive_to_point(self):
        # give the robot a new point to drive to.
        # return false if the robot isn't at the point, and true when it reaches.
        pass

    def update(self, current_pose, dt):
        if self.is_driving:
            # do the driving stuff
            pass
        omega_l = 0.0
        omega_r = 0.0
        return omega_l, omega_r


def compute_jinv(theta):
    # use numpy stuff here too
    pass


def compute_off_center_pt(x, y, theta):
    # use numpy matrices
    pass
