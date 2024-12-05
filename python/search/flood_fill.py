# -*- coding: utf-8 -*-
"""
(BFS)Flood Fill search Algorithm 
Created on Tue Dec  3 13:15:01 2024
@author: Ari
"""
from collections import deque
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient() 
sim = client.require('sim')

def floodFill(img, x, y, newClr, goal=None):
    # Get the color of the starting pixel (prevClr)
    prevClr = img[y][x]  # Note: 'y' is row, 'x' is column (matches the new coordinate system)

    # If the new color is the same as the previous color, no filling is needed
    if prevClr == newClr:
        return

    # Initialize a queue for BFS (Breadth First Search)
    queue = deque([(x, y)])

    # Dictionary to store parent references for path reconstruction
    parent = {(x, y): None}

    # Start filling the region
    img[y][x] = newClr  # Change the starting pixel to the new color

    # Define directions for moving (up, right, down, left)
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # (dx, dy) pairs

    while queue:
        current_x, current_y = queue.popleft()  # Get the current pixel

        # Check if we have reached the goal pixel
        if goal and (current_x, current_y) == goal:
            print(f"Goal has been reached ({current_x},{current_y})")

            # Mark the goal with a distinct color
            img[current_y][current_x] = 5

            # Reconstruct the path from start to goal by following parent links
            path = []
            cell = (current_x, current_y)
            while cell is not None:
                path.append(cell)
                cell = parent[cell]

            # Reverse the path to go from start to goal
            path.reverse()

            # Adjust the path for Cartesian coordinates (bottom-left origin)
            path_cartesian = [(px, len(img) - py - 1) for px, py in path]

            # Filter path to include only coordinates where both x and y are odd
            path_cartesian_odd = [(px, py) for px, py in path_cartesian if px % 2 != 0 and py % 2 != 0]

            print("Path from start to goal (only odd coordinates):", path_cartesian_odd)
            break  # Stop flood fill if the goal is reached

        # Explore all 4 neighboring pixels (up, right, down, left)
        for dx, dy in directions:
            nx, ny = current_x + dx, current_y + dy
            # Check if the new pixel is within bounds and matches the original color
            if 0 <= nx < len(img[0]) and 0 <= ny < len(img) and img[ny][nx] == prevClr:
                img[ny][nx] = newClr  # Change its color to the new color
                queue.append((nx, ny))  # Add the new pixel to the queue for further exploration
                parent[(nx, ny)] = (current_x, current_y)  # Set the parent to current pixel


# Example usage
img = [
    #0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    [1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0],  # y=6
    [1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0],  # y=5
    [1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0],  # y=4
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1],  # y=3
    [1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1],  # y=2
    [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],  # y=1
    [1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1]  # y=0 (origin)
]

goal_pixel = (15, 15)  # Define the goal pixel in (x, y) format
start_pixel = (1, 1)  # Define the starting pixel in (x, y) format

path = floodFill(img, start_pixel[0], start_pixel[1], 3, goal_pixel)  # Start the flood fill from (0, 6)

def sysCall_sensing():
    message = {'id': 'pathPlan','data': path}
    sim.broadcastMsg(message)
    
