import numpy as np
import monitor_utils.transform_utils as t
from pyglet.gl import GL_POINTS
import monitor_utils.running_utils as debugger
import matplotlib.pyplot as plt
from monitor_utils.plotting_utils import PurePursuitPlotter
from scipy.spatial import KDTree
from monitor_utils.transform_utils import FrenetFrame
import monitor_utils.bagging_utils as bagger

"""
speed things up with numba -> njit
"""

class PurePursuitPlanner:
    
    #initialise and read all required inputs (don't loop the planner object)
    def __init__ (self, waypoints, la, vgain, wb):
        #self.conf = conf
        self.waypoints = waypoints
        self.waypointstree = KDTree(waypoints, compact_nodes=True, balanced_tree=True)
        # self.wb = conf.wheelbase 
        self.la = la
        self.vgain = vgain
        self.wb = wb
        global plotter
        plotter = PurePursuitPlotter()
        plotter.plot_data(self.waypoints[:,0], self.waypoints[:,1], 'mo', markersize=1)

        self.frenet = FrenetFrame(bagger.csvtonp('~/STL_workspace/simulator/maps/centerline.csv'), data='raw')

        self.drawn_waypoints =[]

    """
    replace this with kd tree -> either build your own or use scipy -> look at nearest neighbour search on wikipedia
    replace this with f1tenth example to make it faster
    """
    def find_closest_waypoint(self):
        try:
            closest_waypoint_index = 0 #initialise with nonsense index
            min_dist = np.linalg.norm(self.position-self.waypoints[0])
            for i in range(0, self.waypoints.shape[0]-1):
                dist = np.linalg.norm(self.position-self.waypoints[i])
                if dist<min_dist:
                    min_dist=dist
                    closest_waypoint_index = i
            
            assert closest_waypoint_index != -1, "Invalid closest_waypoint_index. Shouldn't be -1"
        except AssertionError as msg:
            print(msg)

        # plotter.plot_data(self.waypoints[:,0], self.waypoints[:,1], 'mo', markersize=1)
        plotter.plot_data(self.position[0,0], self.position[0,1], 'bo', markersize=5)
        plotter.plot_data(self.waypoints[closest_waypoint_index, 0], self.waypoints[closest_waypoint_index, 1], 'ro', markersize=5)

        return min_dist, closest_waypoint_index
    """
    assume you are following along the indices of the track (i.e. that is your notion of direction)
    the row index of the waypoints provides a natural sense of direction with a hash map (dict in python)
    """
    def find_goalpoint(self, wpt_dist, cl_wpt_i):
        if wpt_dist>self.la:
            # print("la_point before"+ str(self.waypoints[cl_wpt_i]))
            return self.waypoints[cl_wpt_i]
        
        else:
            #find closest point within bubble while respecting forward direction
            # dv = self.waypoints[cl_wpt_i+1] - self.waypoints[cl_wpt_i]
            j = 0
            iter_dist = np.linalg.norm(self.waypoints[cl_wpt_i]-self.position)
            while (iter_dist<self.la):
                j = j+1
                #wrap around indices -> to move around track
                if (cl_wpt_i+j >= 804):
                    cl_wpt_i = 0
                    j = 0
                iter_dist = np.linalg.norm(self.waypoints[cl_wpt_i+j]-self.position)
            
            # take projection of required point on to lookahead circle -> need to manage reference frames properly
            la_point = self.waypoints[cl_wpt_i+j] - self.position
            # print("la_point"+ str(la_point))
            la_point_projected = self.la*(la_point/np.linalg.norm(la_point)) + self.position #adding position to adjust for co-ordinate frame

            plotter.plot_data(la_point_projected[0,0], la_point_projected[0,1], 'go', markersize=5)
            plotter.plot_spin()
            #interpolation is more expensive than projection
            return la_point_projected


    #Use this function in rrt planner by providing goalpoint
    def get_actuation(self, goalpoint):
        y = goalpoint[0,1]
        # print('y = '+str(y))
        if np.abs(y) < 1e-6:
            # print('abs selected')
            return self.vgain, 0.
        # radius = 1/(2.0*y/self.la**2)
        # steering_angle = np.arctan(self.wb/radius) #this might not work
        curvature = (2.0*y)/(self.la**2.0)
        # print('curvature: ' + str(curvature))
        # steering_angle = np.clip(curvature, -0.4189, 0.4189)
        steering_angle = np.minimum(0.4189, np.maximum(curvature, -0.4189))
        # print('steering_angle_clipped= '+str(steering_angle))
        debugger.debug_delay(0.05)
        return self.vgain, steering_angle


    #receive all required information and return action commands
    #entry point to this object
    """
    assume heading and pose are numpy arrays
    use transform utils to get frame transformation

    all vectors are row vectors (numpy internally treats them as column vectors)

    NOTE: check out numpy data type!
    """

    def plan(self, heading, position):
        self.heading = heading
        self.position = position
        plotter.plot_data(self.position[0,0], self.position[0,1], 'bo', markersize=5)
        frenetposition = self.frenet.from_global_to_frenet(self.position)
        print(frenetposition)
        # min_dist, closest_waypoint_index = self.find_closest_waypoint()
        min_dist, closest_waypoint_index = self.waypointstree.query(self.position)
        plotter.plot_data(self.waypointstree.data[closest_waypoint_index, 0], self.waypoints[closest_waypoint_index, 1], 'ro', markersize=5)
        goalpoint = self.find_goalpoint(min_dist, closest_waypoint_index)
        rot_m, trans_v = t.from_global_to_car(self.heading, self.position)
        # print(rot_m)
        # # print(rot_m.shape)
        # # print(type(rot_m))
        # # debugger.debug_pause()
        # print(trans_v)
        # # debugger.debug_pause()
        # print(goalpoint.T)
        transformed_goalpoint = (np.dot(rot_m, (goalpoint+trans_v).T)).T #using dot for matrix vector multiplication
        # print(transformed_goalpoint)
        debugger.debug_pause()
        throttle, steering = self.get_actuation(transformed_goalpoint)

        return throttle, steering


    def render_waypoints(self, e):
        """
        update waypoints being drawn by EnvRenderer
        """

        #points = self.waypoints

        points = np.vstack((self.waypoints[:, 0], self.waypoints[:, 1])).T
        
        scaled_points = 50.*points

        for i in range(points.shape[0]):
            if len(self.drawn_waypoints) < points.shape[0]:
                b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [scaled_points[i, 0], scaled_points[i, 1], 0.]),
                                ('c3B/stream', [183, 193, 222]))
                self.drawn_waypoints.append(b)
            else:
                self.drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]

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


"""
points of error:
- closest point
- transformation 
- rendering
and more...
"""

"""
TODO: LEARN PYGLET AND HOW TO RENDER WITH THAT!
Use animation style plotting to debug in matplotlib

waypoint selection is going fine! Great stuff!
"""
"""
continue tomorrow: 
figure out continuous plotting for purepursuit
write latex formalisations 
troubleshoot actuation towards waypoint
tune controller
figure out the different tools you need -> plotting, bagging
collect traces
rrt

wrap in plotter class
"""

"""
ideas to speed up:
- optimised data structures - kd trees and nearest neighbour search on all search operations
- numba njit
- concurrent events and multithreading
- 
"""