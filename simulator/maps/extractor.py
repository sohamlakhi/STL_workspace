import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sympy.solvers import solve
from sympy import Symbol
import time
import os
import monitor_utils.bagging_utils as bagger

"""
Modularise this by turning it into a function!
"""

PATH = os.path.abspath(os.getcwd())

"""
errors:
one point pops out in plot (plot and see which one)
"""
def interp(pt1, pt2):
    x = (pt1[0]+pt2[0])/2
    y = (pt1[1]+pt2[1])/2
    return np.array([[x ,y]])

def get_normal(v):
    a,b = v
    
    x = Symbol('x')
    y = Symbol('y')

    ans = solve((x**2+y**2-1, a*x+b*y), (x,y))
    #write a static assert here to always return 2 answers (or write the different test cases as in circle-line method from purepursuit)
    """
    we're solving a system of non-linear equations here - a circle of radius 1 and a line passing through the origin
    For all parameters, this will ALWAYS yield 2 answers -> which correspond to the normals with both configurations (pointing 'in' and 'out')
    """

    assert len(ans) == 2
    return np.asarray(ans[0]) if (-a/b > 0) else np.asarray(ans[1])

def main():
    pdcenterline = pd.read_csv('IMS_centerline.csv')
    npcl = pdcenterline.to_numpy()

    colsize = npcl.shape[0]
    cl = np.zeros((colsize,2)) #2d arrays of x,y co-ords -> [x y] per row
    inl = np.zeros((colsize,2))
    ol = np.zeros((colsize,2))
    inq = np.zeros((colsize,2))
    oq = np.zeros((colsize,2))

    #using track widths at the ith point (could interpolate to the midpoint -> but it is constant in this case)
    for i in range(0, colsize-1): #looping over all columns
        if i < colsize-1:
            cl[i] = interp(npcl[i], npcl[i+1]) #if i != colsize-1 else interp(npcl[i], npcl[0]) #condition to close the track
            n = get_normal(np.subtract(npcl[i+1], npcl[i])[0:2])

            w_i = npcl[i,2]
            inl[i] = cl[i]-w_i*n
            inq[i] = cl[i]-0.5*w_i*n

            w_o = npcl[i,3]
            ol[i] = cl[i]+w_o*n
            oq[i] = cl[i]+0.5*w_o*n
        
        # else:
        #     cl[i] = interp(npcl[i], npcl[0])
        #     n = get_normal(np.subtract(npcl[0], npcl[i])[0:2])
        #     w_i = npcl[i,2]
        #     inl[i] = cl[i]-w_i*n
        #     inq[i] = cl[i]-0.5*w_i*n

        #     w_o = npcl[i,3]
        #     ol[i] = cl[i]+w_o*n
        #     oq[i] = cl[i]+0.5*w_o*n
    
    #plot centerline
    # plt.plot(npcl[:,0],npcl[:,1], 'bo', markersize=1)
    plt.plot(cl[:,0], cl[:,1],'ro', markersize =1)
    plt.plot(inl[:,0], inl[:,1], 'bo', markersize=1)
    plt.plot(inq[:,0], inq[:,1], 'go', markersize=1)
    plt.plot(ol[:,0], ol[:,1], 'ro', markersize=1)
    plt.plot(oq[:,0], oq[:,1], 'mo', markersize=1)
    plt.axis('equal')
    plt.show()

    #save the stuff - write pandas/numpy implementation 
    bagger.nptocsv(inl, column=['x', 'y'], name='innerline', path=PATH, index_csv = False)
    bagger.nptocsv(ol, column=['x', 'y'], name='outerline', path=PATH, index_csv= False)
    bagger.nptocsv(oq, column=['x', 'y'], name='outerhalf', path=PATH, index_csv= False)
    bagger.nptocsv(inq, column=['x', 'y'], name='innerhalf', path=PATH, index_csv = False)
    bagger.nptocsv(cl, column=['x', 'y'], name='centerline', path=PATH, index_csv = False)


if __name__ == "__main__":
    main()


"""
ways to solve this problem
- find the dv of two points and center point weighted sum (lin comb, average etc.)
- find the normal:
- dot product + unit vector conditions give 2 equations and 2 unknowns -> solve the system symbolically or numerically

OR

- find the centerpoint as above or lerp
- use the point 

just use vector method

WAYS TO SOLVE:
either solve an equation numerically or use a symbolic solver -> symbolic solver uses numerical methods under the hood)
find roots of a given equation (which can be thought of as a point solving two equations)

Can modularise this!

"""

"""
TODO: fix normal orientation problem!!
"""