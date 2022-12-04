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
        self.og = OccupancyGrid(conf =conf, obs=obs, agent=agent)
        self.lidar = Lidar(conf=conf, obs=obs, agent=agent)

        # to clip boundary distance
        self.boundary_list = []
        bl = self.boundary_list
        width = float(self.og.grid_width/(2*self.og.grid_resolution))
        height = float(self.og.grid_height/self.og.grid_resolution)
        theta_threshold = abs(np.arctan(width/height))
        for i in range(0, self.lidar.scan.size-1):
            if (self.lidar.angle_list[i] > self.og.ANGLE_MAX or self.lidar.angle_list[i] < self.og.ANGLE_MIN):
                bl.append(0.0)
                continue
            
            elif self.lidar.angle_list[i] < -theta_threshold:
                bl.append(width/abs(np.sin(self.lidar.angle_list[i])))
            
            elif self.lidar.angle_list[i] > -theta_threshold and self.lidar.angle_list[i] < theta_threshold:
                bl.append(height/abs(np.cos(self.lidar.angle_list[i])))
            
            elif self.lidar.angle_list[i] > theta_threshold:
                bl.append(width/abs(np.sin(self.lidar.angle_list[i])))

        global ocg_fig, ocg_ax
        ocg_fig, ocg_ax = plot.init_plot()

    def populate_ocgrid(self, obs):
        
        self.lidar.update_scan(obs['scans'][0])
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
                """
                clip max distance between that angle and the boundary at that angle. Cache it as a np.array and turn it into a look up problem
                """
                clipped = np.clip(self.lidar.scan[i], 0, self.boundary_list[i])
                
                local_point = np.array([clipped*np.cos(self.lidar.angle_list[i]), clipped*np.sin(self.lidar.angle_list[i])])
                                
                grid_point = self.og.local_to_grid(local_point)
               
                if(clipped<self.boundary_list[i]): 
                    self.og.ocgrid[tuple(grid_point)]=self.og.IS_OCCUPIED

                bline = self.og.bresenham_fill(self.og.local_to_grid(np.array([0,0])).tolist(), grid_point.tolist()) #origin of local reference frame in grid co-ords and endpoint
                for cell in bline:
                    if self.og.ocgrid[tuple(cell)] != self.og.IS_OCCUPIED:
                        self.og.ocgrid[tuple(cell)] = self.og.IS_FREE

        plot.clear_plot(ocg_ax)
        plot.plot_ocgrid(self.og.ocgrid, ocg_ax)

                




"""
go through tree/graph theory and dsa
look at atsuhi's code
look at ahmad's code 
ask leroy for help
"""