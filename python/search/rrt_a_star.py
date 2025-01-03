# -*- coding: utf-8 -*-
"""
Created on Wed Dec  4 15:50:54 2024
RRT for exploring, A_star for path planing
@author: Ari
"""

import heapq
import random
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient() 
sim = client.require('sim')

# Reconstructs the path from the 'cameFrom' map
def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)  # Prepend to reverse the path order
    return total_path


# A* algorithm
def A_star(start, goal, h, d, grid):
    openSet = []
    heapq.heappush(openSet, (0, start))  # (fScore, node)
    cameFrom = {}
    gScore = {start: 0}
    fScore = {start: h(start, goal)}

    while openSet:
        _, current = heapq.heappop(openSet)

        if current == goal:
            path = reconstruct_path(cameFrom, current)
            path_cartesian = [(px, len(grid) - py - 1) for px, py in path]
            filtered_path = [(px, py) for px, py in path_cartesian if px % 2 != 0 and py % 2 != 0]
            return filtered_path

        for neighbor in neighbors(current, grid):
            tentative_gScore = gScore[current] + d(current, neighbor)

            if neighbor not in gScore or tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + h(neighbor, goal)
                heapq.heappush(openSet, (fScore[neighbor], neighbor))

    return None


# Heuristic function for A* (Manhattan distance)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


# Distance function (for grid-based pathfinding)
def distance(current, neighbor):
    return 1


# Get valid neighbors for the robot's configuration (up, down, left, right)
def neighbors(current, grid):
    x, y = current
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Left, Right, Down, Up
    valid_neighbors = []

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if len(grid[0]) > nx >= 0 != grid[ny][nx] and 0 <= ny < len(grid):  # Assuming 0 is blocked
            valid_neighbors.append((nx, ny))

    return valid_neighbors


# RRT function to explore the configuration space and attempt to find the goal
def RRT(start, goal, K, delta_q, grid):
    # Initialize the RRT with the starting point
    G = {start: None}
    for _ in range(K):
        q_rand = (random.randint(0, len(grid[0]) - 1), random.randint(0, len(grid) - 1))
        q_nearest = min(G.keys(), key=lambda q: abs(q[0] - q_rand[0]) + abs(q[1] - q_rand[1]))  # Find nearest vertex

        # Move towards the random point, limiting to delta_q distance
        q_new = (min(q_nearest[0] + delta_q, q_rand[0]), min(q_nearest[1] + delta_q, q_rand[1]))

        # Check if the new point is valid (not blocked)
        if grid[q_new[1]][q_new[0]] != 0:  # Ensure the new point is not blocked
            G[q_new] = q_nearest

            # If we reach a point near the goal, use A* to find the path from start to goal
            if abs(q_new[0] - goal[0]) + abs(q_new[1] - goal[1]) <= delta_q:
                print("Goal reached, refining the path with A*...")
                return A_star(start, goal, heuristic, distance, grid)

    print("RRT did not reach the goal. Pathfinding with A* is not possible.")
    return None


# Example usage
img = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0],
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

start_pixel = (1, 31)
goal_pixel = (15, 15)

# Run RRT to explore space and try to find the goal
path = RRT(start_pixel, goal_pixel, 10000, 2, img)

def sysCall_sensing():
    message = {'id': 'pathPlan','data': path}
    sim.broadcastMsg(message)
    
