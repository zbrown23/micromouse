import time
import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
from controls import Pose2d
from pose_estimator import *


class Robot:
    def __init__(self, sim, wheel_dia=0.032, track_width=0.05):
        self.track_width = track_width
        self.wheel_dia = wheel_dia
        if sim is None:
            raise Exception("need a sim handle!")
        self.sim = sim
        self.l_wheel = self.sim.getObject('/micromouse/l_wheel_joint')
        self.r_wheel = self.sim.getObject('/micromouse/r_wheel_joint')

    def drive(self, v, omega):
        omega_r = -(v + omega * (self.track_width / 2)) / self.wheel_dia
        omega_l = (v - omega * (self.track_width / 2)) / self.wheel_dia
        self.sim.setJointTargetVelocity(self.r_wheel, omega_r)
        self.sim.setJointTargetVelocity(self.l_wheel, omega_l)

    def get_wheel_speeds(self):
        return self.sim.getJointVelocity(self.l_wheel), self.sim.getJointVelocity(self.r_wheel)

    def driveToCell(self, direction, vel):
        pass


def main():
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    robot = Robot(sim)
    initial_position = sim.getObjectPosition(sim.getObject('/micromouse/center_pt'))
    odometry = Odometry(Pose2d(initial_position[0], initial_position[1], 0))
    robot.drive(0.5, 0)
    start_time = sim.getSimulationTime()
    last_time = start_time
    while last_time - start_time < 3:
        now_time = sim.getSimulationTime()
        dt = now_time - last_time
        speeds = robot.get_wheel_speeds()
        odom = odometry.update(speeds[0], speeds[1], dt)
        print(f"{odom}, true position {Pose2d.from_sim(sim.getObjectPose(sim.getObject('/micromouse/center_pt')))}")
        last_time = now_time
    robot.drive(0, 0)


if __name__ == '__main__':
    main()
