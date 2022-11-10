import numpy as np
from geometry_utils import *

#must send geometry pose object when instantiating an occupancy grid
class OccupancyGrid:
    def __init__(self, conf, agent, pose):
        # self.resolution = conf.agents[str(agent)+'_lookahead']
        # self.width = conf.agents[str(agent)+'_']
        # self.height = conf.agents[str(agent)+'_height']
        # self.pose = pose
#need to change above definition
        
        
        



# #can use fast voxel travel algorithm (Ahmad):
# http://www.cse.yorku.ca/~amana/research/grid.pdf
# https://github.com/cgyurgyik/fast-voxel-traversal-algorithm/blob/master/overview/FastVoxelTraversalOverview.md

# #Or use flood fill/bresenham (not sure how bresenham works)
# https://atsushisakai.github.io/PythonRobotics/modules/mapping/lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial.html
