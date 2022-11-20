import numpy as np

#must send geometry pose object when instantiating an occupancy grid
class OccupancyGrid:
    def __init__(self, obs=None, conf=None, agent=None):
        #read these from the config files
        self.grid_resolution = 10
        self.grid_width = 2*3*self.grid_resolution#conf.agents[str(agent)+'_width'] #2 times look-up * resolution 
        self.grid_height = 3*self.grid_resolution#conf.agents[str(agent)+'_height'] #lookup * resolution
        self.IS_OCCUPIED = 1
        self.IS_FREE = 0
        self.ANGLE_MIN = -np.pi/2
        self.ANGLE_MAX = np.pi/2
        self.MAX_RANGE = 3-0.1 #0.1 = 1 cell

        self.init_ocgrid() #initialise ocgrid
        
    
    #call this whenever you want to clear the grid held by that object    
    def init_ocgrid(self):
        self.ocgrid = np.full(shape=(self.grid_height, self.grid_width),fill_value=5, dtype=int)
    
    def reset_ocgrid(self):
        self.ocgrid = np.full(shape=(self.grid_height, self.grid_width),fill_value=5, dtype=int)

    #refer to diagram for these transforms
    def local_to_grid(self, local_point):
        grid_point = np.empty(shape=local_point.shape, dtype=int)
        grid_point[0] = int((self.grid_height-1)-(local_point[0]*self.grid_resolution)) #NOTE: added a '-1' to grid_height to compensate for the indexing out of bounds error
        grid_point[1] = int((-local_point[1]*self.grid_resolution)+self.grid_width//2-1) #NOTE: might have to remove the -1 here
        return grid_point

    def grid_to_local(self, grid_point):
        local_point = np.empty(shape=grid_point.shape)
        local_point[0] = (self.grid_height - grid_point[0])/self.grid_resolution
        local_point[1] = -(grid_point[1] - self.grid_width/2)/self.grid_resolution
        return local_point

    #TODO: speed this up by replacing math with numpy and then decorating with numba
    def bresenham_fill(self, start, end):
        """
        CREDIT: Atsushi Sakai, Python Robotics 
        Draws a line between 2 points on a bitmap
        input: 
        - start: np.array of start point
        - end: np.array of end point

        output:
        returns a np.array of points (cells)

        Reference: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py
                    https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
                    https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf

        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points, dtype=int) #NOTE: dtype = int might cause errors
        return points

    def flood_fill():
        #TODO: take this from atsushi and geeksforgeeks
        #either use recursion (stack) or use a queue
        """
            https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py
            https://www.geeksforgeeks.org/flood-fill-algorithm-implement-fill-paint/
            https://en.wikipedia.org/wiki/Flood_fill
            
        """
        pass





# #can use fast voxel travel algorithm (Ahmad):
# http://www.cse.yorku.ca/~amana/research/grid.pdf
# https://github.com/cgyurgyik/fast-voxel-traversal-algorithm/blob/master/overview/FastVoxelTraversalOverview.md

# #Or use flood fill/bresenham (not sure how bresenham works)
# https://atsushisakai.github.io/PythonRobotics/modules/mapping/lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial.html

# look at this package and use whatever is needed: http://docs.ros.org/en/groovy/api/occupancy_grid_utils/html/coordinate__conversions_8h.html
# look at the above package and its dependencies to understand how to work with occupancy grids
