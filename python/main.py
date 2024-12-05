import time
import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
from controls import Pose2d

cell_spacing = 0.18

class Robot:
    def __init__(self, sim, wheel_dia=0.032, track_width=0.05):
        self.track_width = track_width
        self.wheel_dia = wheel_dia
        if sim is None:
            raise Exception("Need a ZMQ client to drive!")
        self.sim = sim
        self.l_wheel = self.sim.getObject('/micromouse/l_wheel_joint')
        self.r_wheel = self.sim.getObject('/micromouse/r_wheel_joint')

    def drive(self, v, omega):
        omega_r = (v + omega * (self.track_width / 2)) / self.wheel_dia
        omega_l = (v - omega * (self.track_width / 2)) / self.wheel_dia
        self.sim.setJointTargetVelocity(self.r_wheel, -omega_r)
        self.sim.setJointTargetVelocity(self.l_wheel, omega_l)

    def driveToCell(self, direction, vel):
        pass


def main():
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    robot = Robot(sim)
    robot.drive(0, np.pi / 2)
    time.sleep(1)
    robot.drive(0, 0)
    pass


def follow_left_wall(dist, speed):
    pass


def follow_right_wall(dist, vel):
    pass


if __name__ == '__main__':
    main()
