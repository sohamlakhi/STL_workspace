class Lidar:
    def __init__(self, conf = None):
        self.angle_increment = conf.lidar['angle_increment']
        self.angle_min = conf.lidar['angle_min']
        self.angle_max = conf.lidar['angle_max']
        
    #add rad to deg and deg to rad functions. Use njit, numba for this stuff