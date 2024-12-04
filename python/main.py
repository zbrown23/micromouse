import time
from controller import LTVUnicycleController, Robot
import coppeliasim_zmqremoteapi_client as zmq
from util import Pose2d
TIMESTEP_MILLIS = 10


def main():
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    controller = LTVUnicycleController([1, 1, 1], [1, 1], TIMESTEP_MILLIS)
    robot = Robot(sim, controller)
    goal = sim.getObject('/Goal')
    robot.drive(0, 0)
    sim.startSimulation()
    run = False
    while True:
        run = robot.driveToPoint(goal, 0.01)
        time.sleep(TIMESTEP_MILLIS/1000)
        if sim.getSimulationState() == sim.simulation_stopped:
            robot.drive(0, 0)
            exit()


if __name__ == '__main__':
    main()
