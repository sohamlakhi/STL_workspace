import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import monitor_utils.bagging_utils as bagger

PATH = os.path.abspath(os.getcwd())

# pdcenterline = pd.read_csv('IMS_centerline.csv')
# npcl = pdcenterline.to_numpy()

# npcl_strip = np.delete(npcl, slice(None, None, 2), 0)
# npcl_strip = np.delete(npcl_strip, slice(None, None, 2), 0)
# bagger.nptocsv(npcl_strip[:,0:2], column=['x', 'y'], name='poly_centerline_points', path=PATH)

# print(npcl.shape)
# print(npcl_strip.shape)
# plt.plot(npcl[:,0],npcl[:,1], 'bo', markersize=1)
# plt.plot(npcl_strip[:,0],npcl_strip[:,1], 'ro', markersize=1)
# plt.axis('equal')
# plt.show()

# waypoints = np.loadtxt('IMS_raceline.csv', delimiter=';')
# bagger.nptocsv(nparr=waypoints, path=PATH, name='IMS_raceline_formatted')

waypoints = bagger.csvtonp('IMS_raceline_formatted_waypoints.csv')
plt.plot(waypoints[:,0], waypoints[:,1])
plt.axis('equal')
plt.show()