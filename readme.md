# CoppeliaSim Micromouse

# Team Members and Roles
Ari Avalos - Pathfinding algorithm development and optimal route searching

Sophie Ave'Lallemant - Maze design, odometry and perception system

Zach Brown - Robot control, pose estimation, robot design

Adam Thompson - Maze mapping and exploration

# Introduction
Micromouse is a competition where small, autonomous robots navigate through an unknown maze to find and execute route to reach its center in the shortest time possible. The robot starts at a fixed point in the maze and, using various sensors, maps the maze as it explores. Once the maze is mapped, the robot calculates the shortest path to the center and executes the solution. The competition tests various robotics concepts, such as navigation, pathfinding algorithms, sensor integration, and robot control.

Our project aims to develop a Micromouse robot simulation in CoppeliaSim, with the goal of creating a robot capable of navigating an unknown maze, mapping its environment, and calculating and executing the optimal route to the center.
# Approach 

## Maze
We designed our maze according to the current competition rules, featuring a 16x16 grid. The starting point is located in the bottom-left corner, and the robot’s goal is to reach the center 2x2 grid. 
![image](https://github.com/user-attachments/assets/cfde2533-9c36-4ca0-8f5c-8aaf8c60aee9)

## Robot
The robot used for the simulation follows the differential drive design, which is commonly used in competition for its simplicity and effective movement. It is equipped with proximity sensors on the front, left, and right to detect walls, helping it avoid obstacles and stay within the maze pathways. 
![image](https://github.com/user-attachments/assets/de1474dc-6f75-4c5b-8f5b-74d13777b664)



## Robot Control & Odometry
We use an off-center point controller for robot motion. We originally experimented with using a linear time-variant unicycle controller, but found that it was unstable for some desired poses. The code for the LTV controller is still present. In order to explore the maze, the mapping algorithm provides this controller with a new point to go to, which the controller then drives towards. Due to the nature of the hardware involved in the Micromouse competition, we do not have a global knowledge of the pose of the robot, and so we rely on odometry and a gyroscope to figure out our pose. These two measurements are fused with an alpha-beta filter.

## Maze Mapping
As the robot navigates through the maze, it detects and stores information about the maze's structure. Using its sensors, it identifies walls and open paths, gradually building a map of the environment. The robot explores each cell, marking them as visited, and constructs a complete representation of the maze layout.

## Path Calculation
Using the mapping data, the robot calculates the most efficient route to the center. We use a breath first flood fill algorithm, a A* algorithm, and a Rapidly-exploring Random Tree(RRT) search algorithm that is then refined using A* to find the most optimal path for our robot. When one of the algorithms finish their search they then pass a list of postions for the robot to follow.

## Results
At this current moment we have not been able to test how long it takes for each path planning algorithm to complete the maze usin gour simulation. However, if our code was able to pull the master map matrix data from our simulation as we expect it we have theoretical times of completion for all three of the path planning algorithms. We aquired the theoretical times by recreating our master map in a python matrix and running it in our path planing algorithm.
| Algorithm | A-Star| FloodFill | RRT with A-Star|
| time :| 0.002000 seconds | 0.002001 seconds | 0.007002 seconds |


## Conclusion
lorem ipsum
