from monitor_utils.occupancy_grid_utils import OccupancyGrid
from monitor_utils.lidar_utils import Lidar
from monitor_utils.transform_utils import Transformer as t
import monitor_utils.plotting_utils as plot
import numpy as np
import time
from monitor_utils.running_utils import plot_delay, plot_pause

"""
CREDIT: Inspired from Ahmad Amine's (UPenn) RRT implementation on the f1tenth platform
"""

class RRTPlanner:
    def __init__(self, conf = None, obs = None, agent = None):
        self.og = OccupancyGrid(conf, agent)
        self.lidar = Lidar()
        global ocg_fig, ocg_ax
        ocg_fig, ocg_ax = plot.init_plot()

    def populate_ocgrid(self, obs):
        
        self.lidar.update_scan(obs['scans']) # need to fix this api
        self.og.reset_ocgrid()

        #keep track of the axis and figure objects for this instance

        #iterate through the scans to build the grid
        for i in range(0, self.lidar.scan.size-1):
            #skipping angles outside 90-(-90) range. can speed up this condition
            if (self.lidar.angle_list[i] > self.og.ANGLE_MAX or self.lidar.angle_list[i] < self.og.ANGLE_MIN):
                continue

            else:
                #remember your axes have x vertical and y horizontal (car reference frame) 
                #clip scan to within max range for co-ordinate transforms to make sense 
                clipped = np.clip(self.lidar.scan[i], 0, self.og.MAX_RANGE)
                
                local_point = np.array([clipped*np.cos(self.lidar.angle_list[i]), clipped*np.sin(self.lidar.angle_list[i])])
                                
                grid_point = self.og.local_to_grid(local_point)
               
                if(clipped<self.og.MAX_RANGE): 
                    self.og.ocgrid[tuple(grid_point)]=self.og.IS_OCCUPIED

                bline = self.og.bresenham_fill(self.og.local_to_grid(np.array([0,0])).tolist(), grid_point.tolist()) #origin of local reference frame in grid co-ords and endpoint
                for cell in bline:
                    if self.og.ocgrid[tuple(cell)] != self.og.IS_OCCUPIED:
                        self.og.ocgrid[tuple(cell)] = self.og.IS_FREE

        plot.clear_plot(ocg_ax)
        plot.plot_ocgrid(self.og.ocgrid, ocg_ax)

        #plot.close_plot()

    
                




