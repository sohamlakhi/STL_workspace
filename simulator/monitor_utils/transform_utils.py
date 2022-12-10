import numpy as np
from numpy import linalg
import numba as njit
from scipy.spatial.transform import Rotation as R
import time
from scipy.spatial import KDTree
from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import trapezoid
import monitor_utils.bagging_utils as bagger 
import matplotlib.pyplot as plt


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

class FrenetFrame:
    def __init__(self, centerline, s = None, **kwargs):
        self.rot_m = R.from_euler('z', np.pi/2)
        
        # self.centerdata = None
        try:
            if kwargs['data'] == 'raw':
                self.upsample(centerline)
        except:
            pass

        try:
            if kwargs['data'] == 'tree':
                self.centerdata = KDTree(centerline, compact_nodes=True, balanced_tree=True)
                self.s = s
                self.centerdatanp = centerline
        except:
            pass
        
        circle_interp = interp1d(np.array([0,self.s[-1]]), np.array([0, 2*np.pi]))
        self.theta = circle_interp(np.linspace(0, self.s[-1], self.s.size))
        self.radmap = self.s[-1]/(2*np.pi)
        #now theta and s map by index. Since s has equal spacing this makes sense (i.e. linearity is preserved)

        # assert type(self.centerdata) != None, 'frenet frame centerdata not provided! Aborting...'

    def upsample(self, cl):
        differ = np.diff(cl, 1, axis = 0)
        differ = np.append(differ, np.array([cl[0,:]-cl[-1,:]]), axis=0)#append last point minus first point to the end
        differ_norm = np.linalg.norm(differ, axis=1)
        differ_norm_insert_0 = np.insert(differ_norm, 0, 0)
        t = np.cumsum(differ_norm_insert_0)
        cl_new = np.append(cl, np.array([cl[0,:]]), axis=0)

        cs_total = CubicSpline(t, cl_new, bc_type='periodic')

        def integration(cs_total, t_total):

            cs_derivative = cs_total.derivative()

            def integrand(t, i, cs_derivative, t_total_pass):

                def derivative(t,i,index, cs_derivative, t_total_pass):
                    return cs_derivative.c[0,i,index]*((t-t_total_pass[i])**2) + cs_derivative.c[1,i,index]*((t-t_total_pass[i])) + cs_derivative.c[2,i,index]

                x_index = 0
                y_index = 1
                return np.sqrt((derivative(t, i, x_index,cs_derivative, t_total_pass))**2 + (derivative(t, i, y_index, cs_derivative, t_total_pass))**2)

            integral = np.zeros(shape=t_total.shape)
            for id, element in np.ndenumerate(t_total):
                id = id[0]
                assert isinstance(id, int), "index has to be int!"
                if (id == t_total.size-1): break
                # print(id)
                # remember -> you don't need to wrap index around as you already did so when setting up the cubic spline
                interval = np.linspace(t_total[id], t_total[id+1], 100)
                # integral[id+1] = simpson(integrand(interval, id, cs_derivative), interval)
                integral[id+1] = trapezoid(integrand(interval, id, cs_derivative, t_total), interval)
                # print(integral[id+1])

            return integral
        
        piecewise_arclength = integration(cs_total, t)
        s = np.cumsum(piecewise_arclength)

        interp_s_t = interp1d(s,t)

        s_fine = np.linspace(0, s[-1], int(s[-1]/0.001)) #0.001 m between each s point

        t_fine = interp_s_t(s_fine)

        newxy = cs_total(t_fine)
        
        #initialise
        self.s = s_fine
        self.centerdata = KDTree(newxy, compact_nodes=True, balanced_tree=True)
        self.centerdatanp = newxy

    def getnormal(self, i):
        mod = self.s.size - 1
        i = i%mod
        j = (i+1)%mod
        t = np.subtract(self.centerdatanp[j], self.centerdatanp[i])
        t = np.append(t/np.linalg.norm(t), 0)

        return (self.rot_m.apply(t)[0:2]).reshape((1,2))

    """
    in track positive, out track negative --> according to current convention
    indices of s vector and tree data underlying array are the SAME!
    """
    def from_global_to_frenet(self, gposition):
        dist, i = self.centerdata.query(gposition)
        n = self.getnormal(i)
        vec = np.subtract(gposition, self.centerdatanp[i])
        if np.dot(vec.reshape((2,)), (dist*n).reshape((2,)))>0:
            return self.s[i], dist, i
        else:
            return self.s[i], -dist, i

    #account for wrapping half way through. your car might have a phase difference of more than 180 degrees with the other car. Hence, you might want to always use the minimum of 
    # theta or 360-theta. 
    def gap(self, gposition1, gposition2, unwrap = False):
        _, _, i1 = self.from_global_to_frenet(gposition1)
        _, _, i2 = self.from_global_to_frenet(gposition2)
        
        #if unwrap set to false, it autowraps when pose is equal
        dtheta = self.theta[i2]-self.theta[i1]
        if unwrap and dtheta >= np.pi:
            gap = self.radmap*(2*np.pi-dtheta)
        else:
            gap = self.radmap*dtheta

        return gap

    # def from_frenet_to_global(self, fposition):
    #     """
    #     fposition = np.array([[s,d]])
    #     """
    #     x = fposition[0,]


"""
TODO: 
debug gap opertaor
"""
"""
create functions for xy to sd and back
gap operator
"""




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

def test_frenet():
    cl = bagger.csvtonp('~/STL_workspace/simulator/maps/centerline.csv')
    frenet1 = FrenetFrame(cl, data = 'raw')
    frenet2 = FrenetFrame(frenet1.centerdatanp, frenet1.s, data = 'tree')
    print(type(frenet1.centerdata.data))
    plt.plot(frenet1.centerdatanp[:,0], frenet1.centerdatanp[:,1], 'bo', markersize=1)
    # plt.plot(frenet2.centerdatanp[:,0], frenet2.centerdatanp[:,1], 'ro', markersize=1)
    plt.show()

if __name__ == '__main__':
    test_frenet()
    test()
