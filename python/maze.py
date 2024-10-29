import numpy as np
import coppeliasim_zmqremoteapi_client as zmq


# generate a micromouse map in coppeliasim.
# Why spend 2 hours doing something manually when you can spend 10 automating it? :)
# the maze is made up of 18cm x 18cm unit squares arranged to form a 16 x 16 unit grid.
# the walls of these unit cells are 5cm high and 1.2 cm thick.

class Maze:
    def __init__(self, client: zmq.RemoteAPIClient = None):
        self.client = client

    def set_client(self, client: zmq.RemoteAPIClient):
        self.client = client

    def generate_walls(self):
        if self.client is None:
            raise Exception(f"A client is required to generate lattice points.")
        sim = self.client.getObject('sim')
        left_wall = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.012, 2.892, 0.05])
        wall_pose = sim.getObjectPose(left_wall)
        wall_pose[0] = -2.880 / 2
        wall_pose[2] = 0.025
        sim.setObjectPose(left_wall, wall_pose)
        sim.setObjectAlias(left_wall, "left_wall")
        right_wall = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.012, 2.892, 0.05])
        wall_pose = sim.getObjectPose(right_wall)
        wall_pose[0] = 2.880 / 2
        wall_pose[2] = 0.025
        sim.setObjectPose(right_wall, wall_pose)
        sim.setObjectAlias(right_wall, "right_wall")
        top_wall = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [2.868, 0.012, 0.05])
        wall_pose = sim.getObjectPose(top_wall)
        wall_pose[1] = 2.880 / 2
        wall_pose[2] = 0.025
        sim.setObjectPose(top_wall, wall_pose)
        sim.setObjectAlias(top_wall, "top_wall")
        bottom_wall = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [2.868, 0.012, 0.05])
        wall_pose = sim.getObjectPose(bottom_wall)
        wall_pose[1] = -2.880 / 2
        wall_pose[2] = 0.025
        sim.setObjectPose(bottom_wall, wall_pose)
        sim.setObjectAlias(bottom_wall, "bottom_wall")

if __name__ == "__main__":
    client = zmq.RemoteAPIClient()
    maze = Maze(client)
    maze.generate_walls()
