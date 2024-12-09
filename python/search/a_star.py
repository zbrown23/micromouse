import heapq
import coppeliasim_zmqremoteapi_client as zmq 
import numpy as np

# Reconstructs the path from the 'cameFrom' map
def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)  # Prepend to reverse the path order
    return total_path

# A* algorithm
def A_star(h, d):
    #heuristic, distance
    h = 0
    d = 0
    #Get the Master map from copeliasim and assign it as img
    client = zmq.RemoteAPIClient() 
    sim = client.getObject('sim')
    script = sim.getObjectHandle('/Script')
    MasterMap =  sim.callScriptFunction('getMap', script)
    if MasterMap is None:
        print("No map available, returned None.")
     #Further processing on img (assuming it's a valid object)
        if MasterMap:
            grid = []
            grid = np.array(img)
    # Hardcoded start at (1, 1)
    start = (1, 1)

    # Calculate the goal as the center of the grid
    height, width = len(grid), len(grid[0])
    goal = (width // 2, height // 2)  # Goal is at the center of the grid

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

            # Only include coordinates where both x and y are odd
            odd_path = [(px, py) for px, py in path if px % 2 != 0 and py % 2 != 0]
            
            # Adjust the odd positions by applying (x + 1) // 2 and (y + 1) // 2
            adjusted_path = [( (px + 1) // 2, (py + 1) // 2 ) for px, py in odd_path]
            
            # Return the adjusted path
            return adjusted_path

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
        if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid) and grid[ny][nx] != 0:  # Assuming 0 is a blocked cell
            valid_neighbors.append((nx, ny))

    return valid_neighbors
