"""
to make frenet frame_tests
"""
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import monitor_utils.bagging_utils as bagger 
import numpy as np


# x = np.arange(10)
# y = np.sin(x)

# cs = CubicSpline(x,y)
# print(cs)
# print(type(cs))
# xs = np.arange(-0.5, 9.6, 0.1)
# print(type(cs(xs)))
# print(cs.x)
# print(cs.c)

"""
class attributes of scipy CubicSpline contain the piecewise poly data and t attributes
see atsushi's implementation
"""



# x = np.array([[1, 2], [8, 7], [4,10]])
# difference = np.diff(x, 1, axis = 0)
# print(difference)

# print(difference)
# diff_norm = np.linalg.norm(difference, axis=1)
# print(diff_norm)
# diff_insert_0 = np.insert(diff_norm, 0, 0)
# print(diff_insert_0)
# t = np.cumsum(diff_insert_0) 
# t_col = t[:, np.newaxis]
# print(t)
# print(t_col)
#note: different axes for different oeperations
cl = bagger.csvtonp('~/STL_workspace/simulator/maps/centerline.csv')
differ = np.diff(cl, 1, axis = 0)
differ = np.append(differ, cl[0,:]-cl[-1,:], axis=0)#append last point minus first point to the end
differ_norm = np.linalg.norm(differ, axis=1)
differ_norm_insert_0 = np.insert(differ_norm, 0, 0)
t = np.cumsum(differ_norm_insert_0)
# t = t[:, np.newaxis]
cl_new = np.append(cl, cl[0,1], axis=0)

print(t.shape)
print(cl[:,0].shape)
cs_x = CubicSpline(t, cl[:,0])
cs_y = CubicSpline(t, cl[:,1])

t_plot = np.linspace(0, 20)

plt.plot(cs_x(t_plot), cs_y(t_plot))
plt.show()