import numpy as np
from scipy.spatial.transform import Rotation as R

class Point:
    def __init__(self, x = None, y = None, z = None):
        self.x = float(x)
        self.y = float(y)
    
    def get_nparray(self):
        return np.array([self.x, self.y])

class Orientation:
    def __init__(self, heading = None):
        self.heading = heading
    
    def rot_mat(self):
        return R.from_euler('z', self.heading)

class Pose:
    def __init__(self, point=None, orientation=None):
        self.point = point
        self.orientation = orientation