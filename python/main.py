import time
import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
from controls import Pose2d
from pose_estimator import *
from off_center_controller import OffCenterController


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


def main():
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    robot = Robot(sim)
    initial_position = sim.getObjectPosition(sim.getObject('/micromouse/center_pt'))
    l_wheel_joint = sim.getObject('/micromouse/l_wheel_joint')
    r_wheel_joint = sim.getObject('/micromouse/r_wheel_joint')
    controller = OffCenterController(sim, 1, 0.032, 0.05, 0.01)
    start_time = sim.getSimulationTime()
    last_time = start_time
    while last_time - start_time < 3:
        u = controller.update(Pose2d.from_sim(sim.getObjectPose(sim.getObject('/micromouse/center_pt'))), Pose2d.from_sim(sim.getObjectPose(sim.getObject('/Dummy'))))
        
        if u[0] < -0.5 and u[1] > 0.5:
            u = [-6, -6]
        
        sim.setJointTargetVelocity(l_wheel_joint, u[0])
        sim.setJointTargetVelocity(r_wheel_joint, u[1])


if __name__ == '__main__':
    main()
