function sysCall_init()
    sim = require('sim')

    -- do some initialization here
    grid = Matrix(16,16,{})
    hor_walls = Matrix(15,15) -1
    vert_walls = Matrix(15,15) -1

    grid_size = 10 --inches
    pos_uncertainty = 0.15 * grid_size -- within 0.15 tiles +- of center
    h_uncertainty = 10/180 *math.pi --within 10 degrees +- of target
    clear_thresh = 7.5 --inches
    NORTH = 1
    EAST = 0
    SOUTH = 3
    WEST = 2
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- Get the current position, orientation, and wall readings
    curr_location = get_position()
    curr_heading = get_orientation()

    -- asign current position to grid, or if uncertain take no action
    x = snapToGrid(curr_location[1])
    y = snapToGrid(curr_location[2])

    -- assign current orientation to one of 4 (if uncertain take no action):
    -- NORTH, EAST, SOUTH, WEST
    orientation = snapOrientation(curr_heading)

    -- if all values are griddized, collect data about this square
    if (x != -1 && y != -1 && orientation != -1)
        -- Get values from LiDAR to determine if there is a wall or not
        lidar_vals = getLidar()
        front = lidar_vals[1]
        left = lidar_vals[2]
        right = lidar_vals[3]
        -- Update wall values
        if (orientation = EAST) -- Facing EAST
            vert_walls[x,y] = front
            hor_walls[x,y] = left
            hor_walls[x,y-1] = right
        else if (orientation = NORTH) -- Facing NORTH
            hor_walls[x,y] = front
            vert_walls[x,y] = right
            vert_walls[x-1,y] = left
        else if (orientation = WEST) -- Facing WEST
            vert_walls[x-1,y] = front
            hor_walls[x,y] = right
            hor_walls[x,y-1] = left
        else -- Facing SOUTH
            hor_walls[x,y-1] = front
            vert_walls[x-1,y] = right
            vert_walls[x,y] = left
        end
    end
    -- Start of the flood fill algorithm
    -- Goal: take every path
    -- general premise: survey options. When a decision must be made (i.e. multiple
    -- paths available, store this cell and return to it then make decision)
    -- will do this with stacking. Each cell will enter the stack, and once all
    -- options are depleted it will enter the closed list and no longer generate
    -- options when it is reached.
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
function round_up(num)
    if (num >0)
        return math.ceil(num)
    else
        return math.floor(num)
    end
end

function get_position()
    -- get raw position data from the robot, then subtract the values for 
    x = 5.0
    y = 5.0
    return [x, y]
end

function get_orientation()
    h = math.pi/2
    return h
end

function snapToGrid(val)
    full_tiles = math.floor(val/grid_size)
    remainder = val - full_tiles*grid_size
    if (remainder <= pos_uncertainty)
        return full_tiles
    else if(remainder >= (1-pos_uncertainty)
        return full_tiles + 1
    else
        return -1
    end
end

function snapOrientation(val)
    q_turns = math.floor(val/(math.pi/2))
    remainder = val - q_turns*math.pi/2
    if (remainder <= h_uncertainty)
        return q_turns
    else if (remainder >= (math.pi/2 - h_uncertainty)
        return q_turns + 1
    else
        return -1
    end
end

function getLidar()
    front = 20
    left = 5
    right = 3
    -- Update all values such if its clear, its 1, otherwise its 0
    front = front <= clear_thresh ? 0 : 1
    left = left <= clear_thresh ? 0 : 1
    right = right <= clear_thresh ? 0 : 1
    return [front, left, right]
end