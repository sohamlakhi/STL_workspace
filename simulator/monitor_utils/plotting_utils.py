#convenient matplotlib functions and abstractions to quickly get what you need (look at plt lifecycle to understand this)
import matplotlib.pyplot as plt
import numpy as np
from monitor_utils.running_utils import plot_delay, plot_pause
"""
    Figure out how to use animations for the occupancy grid animation
    https://matplotlib.org/stable/api/animation_api.html
    https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
    
"""

#TODO: redesign this entire API and correspondingly change where it is being used

# initialises plot and returns a global axis
def init_plot():
    global fig
    fig, ax = plt.subplots()
    return fig, ax

def clear_plot(ax):
    ax.clear()

def plot_ocgrid(ocgrid, ax):
    ax.imshow(ocgrid)
    ax.set_aspect('equal')
    plt.pause(0.1)
    #plt.show(block=False)
    

def close_plot():
    plt.close()


# use something like this to do your animations or use matplotlib animations: https://matplotlib.org/stable/gallery/animation/animation_demo.html#sphx-glr-gallery-animation-animation-demo-py
# blit this to speed it up instead of clearing and plotting again and again  https://matplotlib.org/stable/gallery/event_handling/pong_sgskip.html#sphx-glr-gallery-event-handling-pong-sgskip-py
# https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
# look at atsushi RRT implementation code 