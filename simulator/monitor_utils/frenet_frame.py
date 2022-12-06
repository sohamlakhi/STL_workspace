"""
to make frenet frame_tests
"""
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import monitor_utils.bagging_utils as bagger 
import numpy as np
from scipy.integrate import trapezoid, simpson


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
cl = bagger.csvtonp('~/STL_workspace/simulator/maps/poly_centerline_points - poly_centerline_points.csv')
# print(cl.shape)
differ = np.diff(cl, 1, axis = 0)
# print(differ.shape)
differ = np.append(differ, np.array([cl[0,:]-cl[-1,:]]), axis=0)#append last point minus first point to the end
print(differ.shape)
# print(differ.shape)
differ_norm = np.linalg.norm(differ, axis=1)
# print(differ_norm.shape)
differ_norm_insert_0 = np.insert(differ_norm, 0, 0)
# print(differ_norm_insert_0.shape)
t = np.cumsum(differ_norm_insert_0)
# t = t[:, np.newaxis]
# print(t.shape)
cl_new = np.append(cl, np.array([cl[0,:]]), axis=0)
print(cl_new.shape)

# print(t.shape)
# print(cl[:,0].shape)
print('sdf')
print(cl_new[:,0].shape)
cs_x = CubicSpline(t, cl_new[:,0], bc_type = 'periodic')
cs_y = CubicSpline(t, cl_new[:,1], bc_type = 'periodic')

cs_total = CubicSpline(t, cl_new, bc_type='periodic')

t_plot = np.linspace(0, t[-1] ,t.shape[0])

# print(cs_x.c.shape)
print(cs_total.c.shape)
# print(cs_total.c)

# print(cs_x.c[0,0])
# print(cs_total.c[0,0,0])

# assert cs_x.c[0,0] == cs_total.c[0,0,0]

# plt.plot(cs_x(t_plot), cs_y(t_plot), 'r', label = 'separate')
# plt.plot(cs_total(t_plot)[:,0], cs_total(t_plot)[:,1], 'b', label='together')
# plt.axis('equal')
# plt.pause(1)

# print(cs_total.c.shape)
# #test derivative
# cs_derivative = cs_total.derivative(1)
# print(cs_derivative.c.shape)

# print(3*cs_total.c[0,0,0])
# print(cs_derivative.c[0,0,0])

# assert 3*cs_total.c[0,0,0] == cs_derivative.c[0,0,0]

# # integrating arc length
# x_index = 0
# y_index = 1
# cs_derivative = cs_total.derivative()
# sub_integrand_x = cs_derivative.c[0,i,x_index]*(t**2) + cs_derivative.c[1,i,x_index]*(t) + cs_derivative.c[2,i,x_index]
# sub_integrand_y = cs_derivative.c[0,i,y_index]*(t**2) + cs_derivative.c[1,i,y_index]*(t) + cs_derivative.c[2,i,y_index]
# integrand = np.sqrt((sub_integrand_x)**2+(sub_integrand_y)**2)

#using function objects

#this function is old (doesn't account for t scaling)
"""def integration(cs_total, t_total):

    cs_derivative = cs_total.derivative()

    def integrand(t, i, cs_derivative):

        def derivative(t,i,index, cs_derivative):
            return cs_derivative.c[0,i,index]*(t**2) + cs_derivative.c[1,i,index]*(t) + cs_derivative.c[2,i,index]

        x_index = 0
        y_index = 1
        return np.sqrt((derivative(t, i, x_index,cs_derivative))**2 + (derivative(t, i, y_index, cs_derivative))**2)

    # for i in range(0, t.shape[0] - 2):
    integral = np.zeros(shape=t_total.shape)
    for id, element in np.ndenumerate(t_total):
        id = id[0]
        assert isinstance(id, int), "index has to be int!"
        if (id == t_total.size-1): break
        print(id)
        # remember -> you don't need to wrap index around as you already did so when setting up the cubic spline
        interval = np.linspace(t_total[id], t_total[id+1], 100)
        # integral[id+1] = simpson(integrand(interval, id, cs_derivative), interval)
        integral[id+1] = trapezoid(integrand(interval, id, cs_derivative), interval)
        print(integral[id+1])

    return integral"""

#the ppoly instance stores t_total (from when it was constructed)
#this function accounts for t correction
def integration(cs_total, t_total):

    cs_derivative = cs_total.derivative()

    def integrand(t, i, cs_derivative, t_total_pass):

        def derivative(t,i,index, cs_derivative, t_total_pass):
            return cs_derivative.c[0,i,index]*((t-t_total_pass[i])**2) + cs_derivative.c[1,i,index]*((t-t_total_pass[i])) + cs_derivative.c[2,i,index]

        x_index = 0
        y_index = 1
        return np.sqrt((derivative(t, i, x_index,cs_derivative, t_total_pass))**2 + (derivative(t, i, y_index, cs_derivative, t_total_pass))**2)

    # for i in range(0, t.shape[0] - 2):
    integral = np.zeros(shape=t_total.shape)
    for id, element in np.ndenumerate(t_total):
        id = id[0]
        assert isinstance(id, int), "index has to be int!"
        if (id == t_total.size-1): break
        print(id)
        # remember -> you don't need to wrap index around as you already did so when setting up the cubic spline
        interval = np.linspace(t_total[id], t_total[id+1], 100)
        # integral[id+1] = simpson(integrand(interval, id, cs_derivative), interval)
        integral[id+1] = trapezoid(integrand(interval, id, cs_derivative, t_total), interval)
        print(integral[id+1])

    return integral
interdone = integration(cs_total, t)
print(interdone)



# theta = 2 * np.pi * np.linspace(0, 1, 5)
# y = np.c_[np.cos(theta), np.sin(theta)]
# cs_test = CubicSpline(theta, y, bc_type='periodic')
# interdone = integration(cs_test, theta)


# plt.plot(cs_x(t_plot), cs_y(t_plot), 'r', label = 'separate')
# plt.plot(cs_total(t_plot)[:,0], cs_total(t_plot)[:,1], 'b', label='together')
# plt.plot(theta, interdone, 'bo', label = "piecewise integral")
# plt.plot(theta, cs_test(theta)[:,0], 'ro', label = "x vs t")
# plt.plot(theta, cs_test(theta)[:,1], 'mo', label = 'y vs t')
# plt.xlabel('theta')
# plt.axis('equal')
# plt.legend()
# # plt.pause(1)
# plt.show()

x_plotting = np.arange(0,t.size, 1)
print(np.cumsum(interdone)[-1])
print(t[-1])
plt.plot(x_plotting, t, '-r')
plt.plot(x_plotting, np.cumsum(interdone), '-b')
print('break')
print(t.shape)
print(np.cumsum(interdone).shape)
# plt.plot(t, interdone, 'bo', label = "piecewise integral")
plt.show()


"""
param curve by: ti - ti-1 = ||pi-pi-1||
steps to parameterise curve:
- take the difference of consecutive points, including first and last point
- take the norm of each difference
- cumulatively sum the differences with the first point being 0 (append array)
- append the line with the last point
- cubic spline interpolate t and points
"""

"""
cubic spline is an inherited class from ppoly. Basically sets up cubic spline interpolation
You can use a ppoly instance and do it yourself
Ppoly has __call__ functions and member functions like integrate, differentiate etc. -> see how you can use them
the c array member of the ppoly instance contains the piecewise polynomials. It is (4 x n-1 x m) -> contains 4 (axis=0) (a, b, c, d), n-1 (axis=1) for all the piecewise polynomials and m (axis=2) dependent variables

NOTE: look at cubic interpolation matrices again

"""