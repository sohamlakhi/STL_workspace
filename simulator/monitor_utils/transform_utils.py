import numpy as np
from numpy import linalg
import numba as njit
from scipy.spatial.transform import Rotation as R

#speed up these transforms using numba (look at caching) and memoization
class Transformer:
    def __init__(self) -> None:
        pass
    
    #get cartesian transforms to and from global and local reference frames (i.e. all agents)
    def from_global_to_car(self, heading, position):
        rot_m = R.from_euler('z', heading)
        trans_v = position
        return np.asarray(rot_m), trans_v

    def from_car_to_global(self, heading, position):
        rot_mat, trans_v = self.get_cartesian_transform(heading, position)
        return linalg.inv(rot_mat), -trans_v

    def frompi2pi(self, angle):
        if angle<0:
            return 2*np.pi-angle
        else:
            return angle

    def topi2pi(self, angle):
        if angle>np.pi:
            return angle-2*np.pi
        else:
            return angle

    #add bezier transform utilities here

    #add clothoid utilities here

