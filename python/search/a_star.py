# -*- coding: utf-8 -*-
"""
Created on Tue Dec  3 14:08:54 2024

@author: Fulge
"""
import heapq

# Reconstructs the path from the 'cameFrom' map
def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)  # Prepend to reverse the path order
    return total_path

# A* algorithm
def A_star(start, goal, h, d, grid):
    # The set of discovered nodes that may need to be (re-)expanded
    openSet = []
    heapq.heappush(openSet, (0, start))  # (fScore, node)

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n
    cameFrom = {}

    # For node n, gScore[n] is the currently known cost of the cheapest path from start to n
    gScore = {start: 0}

    # For node n, fScore[n] is the best estimate of the cheapest path from start to goal passing through n
    fScore = {start: h(start, goal)}

    while openSet:
        # Get the node with the lowest fScore
        _, current = heapq.heappop(openSet)

        # If the current node is the goal, reconstruct and return the path
        if current == goal:
            path = reconstruct_path(cameFrom, current)
            # Convert the path to Cartesian coordinates (origin at bottom-left)
            path_cartesian = [(px, len(grid) - py - 1) for px, py in path]
            
            # Only include coordinates where either x or y is odd
            filtered_path = [(px, py) for px, py in path_cartesian if px % 2 != 0 and py % 2 != 0]
            return filtered_path

        # Explore neighbors
        for neighbor in neighbors(current, grid):
            tentative_gScore = gScore[current] + d(current, neighbor)

            if neighbor not in gScore or tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + h(neighbor, goal)
                heapq.heappush(openSet, (fScore[neighbor], neighbor))

    # If no path is found, return failure
    return None

# Heuristic function (Manhattan distance for grid-based pathfinding)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Distance function (for grid-based pathfinding, assuming uniform cost per move)
def distance(current, neighbor):
    return 1  # Each move has the same cost

# Function to get valid neighbors (up, down, left, right)
def neighbors(current, grid):
    x, y = current
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Left, Right, Down, Up
    valid_neighbors = []
    
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if len(grid[0]) > nx >= 0 != grid[ny][nx] and 0 <= ny < len(grid):  # Assuming 0 is a blocked cell
            valid_neighbors.append((nx, ny))

    return valid_neighbors

# Example usage
img = [
    #0, 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],#0
    [0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],#1
    [0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],#2
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0],#3
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],#4
    [0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0],#5
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0],#6
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0],#7
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],#8
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],#9
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #10
]

start_pixel = (1, 9)  # Starting point (x, y) in (column, row)
goal_pixel = (17, 5)  # Goal point (x, y) in (column, row)

# Run A* to find the path
path = A_star(start_pixel, goal_pixel, heuristic, distance, img)

# Output the path
if path:
    print("Path from start to goal (Cartesian coordinates with bottom-left origin, filtering even cells):")
    print(path)
else:
    print("No path found.")