import numpy as np
from monitor_utils.transform_utils import Transformer as t

class PurePursuitPlanner:
    
    #initialise and read all required inputs (don't loop the planner object)
    def __init__ (self, waypoints, la, vgain):
        #self.conf = conf
        self.waypoints = waypoints
        # self.wb = conf.wheelbase 
        self.la = la
        self.vgain = vgain


    def find_closest_waypoint(self):
        
        return np.array#can use lambda here
        pass
    
    def p_controller(self):
        pass

    #receive all required information and return action commands
    def plan(self, heading, position):
        #receive pose
        #call find closest waypoint (return it in the car's ref frame)

        #call p_controller (project or do whatever with the point)
        pass
        #return throttle, steering

"""
    steps:
    - timer tick
    - receive pose
    - find closest point within lookahead locus (in world reference frame)
    - construct transformation matrix (from heading relative to globe) and translation vector
    - transform selected waypoint to car's reference frame
    - p controller on steering (within reason commands)
    - get appropriate action commands and return 

    https://github.com/f1tenth/f1tenth_planning: look at this repo for help
    it has stuff on clothoids as well, that can help you with your bezier curve stuff (and rrt -> look at resources for rrt)

"""
