from collections import deque
import coppeliasim_zmqremoteapi_client as zmq 

def floodFill():
    
    # Get the Master map from copeliasim and assign it as img
    client = zmq.RemoteAPIClient() 
    sim = client.getObject('sim')
    script = sim.getObjectHandle('/Sript')
    img =  sim.callScriptFunction('getMap', script)
     if MasterMap is None:
        print("No map available, returned None.")
    # Further processing on img (assuming it's a valid object)
        if MasterMap:
            img = []
            img = np.array(img)
    #setting the starting cell
    x = 0
    y = 0
    # setting the pathing pixel
    newClr = 3
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
    directions = [(1, 0), (0, -1), (-1, 0), (0, 1)]  # (dx, dy) pairs
    
    # Calculate the goal position (center of the image)
    goal_x = len(img[0]) // 2  # Center column
    goal_y = len(img) // 2      # Center row
    goal_pixel = (goal_x, goal_y)  # Define the goal pixel in (x, y) format
    
    while queue:
        current_x, current_y = queue.popleft()  # Get the current pixel
        
        # Check if we have reached the goal pixel
        if (current_x, current_y) == goal_pixel:
            # Mark the goal with a distinct color
            img[current_y][current_x] = 5
            
            # Reconstruct the path from goal to start by following parent links
            path = []
            cell = (current_x, current_y)
            while cell is not None:
                path.append(cell)
                cell = parent[cell]
            
            # Reverse the path to go from start to goal
            path.reverse()
            
            # Mark the path from goal to start with color 4, but only include odd positions
            final_path = []
            for px, py in path:
                if px % 2 != 0 and py % 2 != 0:  # Only process odd positions
                    if img[py][px] != 5:  # Avoid overwriting the goal marker (5)
                        img[py][px] = 4
                        # Apply transformation to odd positions
                        new_x = (px + 1) // 2
                        new_y = (py + 1) // 2
                        final_path.append((new_x, new_y))

            return final_path
        
        # Explore all 4 neighboring pixels (up, right, down, left)
        for dx, dy in directions:
            nx, ny = current_x + dx, current_y + dy
            # Check if the new pixel is within bounds and matches the original color
            if 0 <= nx < len(img[0]) and 0 <= ny < len(img) and img[ny][nx] == prevClr:
                img[ny][nx] = newClr  # Change its color to the new color
                queue.append((nx, ny))  # Add the new pixel to the queue for further exploration
                parent[(nx, ny)] = (current_x, current_y)  # Set the parent to current pixel

