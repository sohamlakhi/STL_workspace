import numpy as np
from numpy import linalg
import numba as njit
from scipy.spatial.transform import Rotation as R
import time
#speed up these transforms using numba (look at caching) and memoization

#get cartesian transforms to and from global and local reference frames (i.e. all agents)
#REMEMBER -> you're transforming and translating reference frames -> accordingly, your rotations should be passive and translations should be signed correctly
def from_global_to_car(heading, position):
    rot_m = R.from_euler('z', -heading[0])
    rot_m = rot_m.as_matrix()[0:2,0:2]
    trans_v = -position
    return rot_m, trans_v #return only 2 by 2 matrix

def from_car_to_global(heading, position):
    rot_mat, trans_v = from_global_to_car(heading, position)
    return linalg.inv(rot_mat), -trans_v

def frompi2pi(angle):
    if angle<0:
        return 2*np.pi-angle
    else:
        return angle

def topi2pi(angle):
    if angle>np.pi:
        return angle-2*np.pi
    else:
        return angle

#add bezier transform utilities here

#add clothoid utilities here

#add track based utilities here

"""
use pytest or unitest to do this!!!!
"""

def test():
    position = np.array([[1, 2]])
    rot_m, trans_v = from_global_to_car(np.array([np.pi/4]), position)

    goalpoint = np.array([[3,4]])

    transformed = (np.dot(rot_m,goalpoint.T)).T+trans_v
    transformed = (np.dot(rot_m, (goalpoint+trans_v).T)).T
    print(rot_m)
    print(trans_v)
    print(transformed)

    # print(np.asarray(rot_m))
    # print(rot_m.shape)
    # print(trans_v)
    # print(trans_v.shape)


if __name__ == '__main__':
    test()
