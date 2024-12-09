import time
import numpy as np
import coppeliasim_zmqremoteapi_client as zmq
from controls import Pose2d, PIDController
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
        return omega_l, omega_r

    def get_wheel_speeds(self):
        return self.sim.getJointVelocity(self.l_wheel), self.sim.getJointVelocity(self.r_wheel)


def main():
    solved = False
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    sim_time = sim.getSimulationTime()
    robot = Robot(sim)
    dummy = sim.getObject('/Dummy')
    mouse_center = sim.getObject('/micromouse/center_pt')
    l_wheel_joint = sim.getObject('/micromouse/l_wheel_joint')
    r_wheel_joint = sim.getObject('/micromouse/r_wheel_joint')
    controller = OffCenterController(sim, 1, 0.032, 0.05, 0.01)
    heading_controller = PIDController(1, 0, 0)
    while not solved:
        dt = sim.getSimulationTime() - sim_time
        sim_time = sim.getSimulationTime()
        robot_pose = Pose2d.from_sim(sim.getObjectPose(mouse_center))
        target_pose = Pose2d.from_sim(sim.getObjectPose(dummy))
        u = controller.update(robot_pose, target_pose)
        heading_error = target_pose.theta - robot_pose.theta
        omega_effort = heading_controller.update(heading_error, robot_pose.theta)
        heading_u = robot.drive(0, omega_effort)
        sim.setJointTargetVelocity(l_wheel_joint, u[0] + heading_u[0])
        sim.setJointTargetVelocity(r_wheel_joint, u[1] + heading_u[1])


if __name__ == '__main__':
    main()
