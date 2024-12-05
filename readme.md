# CoppeliaSim Micromouse

# Team Members and Roles
Ari Avalos - Pathfinding algorithm development and optimal route searching

Sophie Ave'Lallemant - Maze design, odometry and perception system

Zach Brown - Robot control and movement execution

Adam Thompson - Maze mapping and exploration

# Introduction
Micromouse is a competition where small, autonomous robots navigate through an unknown maze to find and execute route to reach its center in the shortest time possible. The robot starts at a fixed point in the maze and, using various sensors, maps the maze as it explores. Once the maze is mapped, the robot calculates the shortest path to the center and executes the solution. The competition tests various robotics concepts, such as navigation, pathfinding algorithms, sensor integration, and robot control.

Our project aims to develop a Micromouse robot simulation in CoppeliaSim, with the goal of creating a robot capable of navigating an unknown maze, mapping its environment, and calculating and executing the optimal route to the center.
# Approach 

## Maze
We designed our maze according to the current competition rules, featuring a 16x16 grid. The starting point is located in the bottom-left corner, and the robot’s goal is to reach the center 2x2 grid. 
![image](https://github.com/user-attachments/assets/cfde2533-9c36-4ca0-8f5c-8aaf8c60aee9)

## Robot
The robot used for the simulation follows the differential drive design, which is commonly used in competition for its simplicity and effective movement. It is equipped with proximity sensors on the front, left, and right to detect walls, helping it avoid obstacles and stay within the maze pathways. Odometry is utilized to estimate the robot’s position within the maze, enabling it to track its movements and make accurate decisions about where to go next.

## Robot Control & Odometry

## Maze Mapping
As the robot navigates through the maze, it detects and stores information about the maze's structure. Using its sensors, it identifies walls and open paths, gradually building a map of the environment. The robot explores each cell, marking them as visited, and constructs a complete representation of the maze layout.

## Path Calculation
Using the mapping data, the robot calculates the most efficient route to the center. We use a flood fill algorithm and A* to calculate the optimal path to the goal.


## Results
lorem ipsum

## Conclusion
lorem ipsum
