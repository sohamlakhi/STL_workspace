import numpy as np

"""
    TODO:
    - restructure this API or fix it to something more convenient
"""

class Lidar:
    def __init__(self, obs=None, conf=None, agent=None):
        # read from config file
        self.angle_increment =0.00435588507 #conf.lidar['angle_increment']
        self.angle_min = -2.35#conf.lidar['angle_min']
        self.angle_max = 2.35#conf.lidar['angle_max']
        self.scan = obs['scans'][0] #change API to deal with role reversal

        self.angle_list = np.empty(shape = self.scan.shape)
        for i in range(0, self.scan.size-1):
            self.angle_list[i] = self.get_angle(i)

        
    def get_angle(self, i):
        return (self.angle_min + i*self.angle_increment)

    def get_range_index(self, angle):
        return ((angle-self.angle_min)/self.angle_increment)

    def update_scan(self, scan):
        self.scan = scan

        # self.angle_list = np.empty(shape = self.scan.shape)
        # for i in range(0, self.scan.size-1):
        #     self.angle_list[i] = self.get_angle(i)

    #add rad to deg and deg to rad functions. Use njit, numba for this stuff
