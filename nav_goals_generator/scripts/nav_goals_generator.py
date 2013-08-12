#!/usr/bin/env python
import roslib; roslib.load_manifest('nav_goals_generator')
import rospy
import tf

from nav_goals_msgs.srv import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid

import random
import math
import numpy
import sys

################################################################
# Ray-casting algorithm
# 
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
_eps = 0.00001

def ray_intersect_seg(p, a, b):
    ''' takes a point p and an edge of two endpoints a,b of a line segment returns boolean
    '''
    if a.y > b.y:
        a,b = b,a
    if p.y == a.y or p.y == b.y:
        p = Point32(p.x, p.y + _eps, 0)
 
    intersect = False
 
    if (p.y > b.y or p.y < a.y) or (
        p.x > max(a.x, b.x)):
        return False
 
    if p.x < min(a.x, b.x):
        intersect = True
    else:
        if abs(a.x - b.x) > sys.float_info.min:
            m_red = (b.y - a.y) / float(b.x - a.x)
        else:
            m_red = sys.float_info.max
        if abs(a.x - p.x) > sys.float_info.min:
            m_blue = (p.y - a.y) / float(p.x - a.x)
        else:
            m_blue = sys.float_info.max
        intersect = m_blue >= m_red
    return intersect
 
def is_odd(x): return x%2 == 1
 
def is_inside(p, poly):
    ln = len(poly)
    num_of_intersections = 0
    for i in range(0,ln):
        num_of_intersections += ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])

    return is_odd(num_of_intersections)

################################################################
# Navigation goal generator
################################################################

class NavGoalsGenerator():
	"A class for generation random poses for the robot"

	def __init__(self):
            rospy.init_node('nav_goals_generator')
            rospy.loginfo("Started nav_goals_generator service")

            # subscribing to a map
            self.map_frame = rospy.get_param('map_frame', '/map')
            rospy.Subscriber(self.map_frame, OccupancyGrid, self.map_callback)

            # setting up the service
            self.ser = rospy.Service('/nav_goals', NavGoals, self.nav_goals_service)

            # visualizing nav goals in RVIZ
            self.pub = rospy.Publisher('nav_goals', PoseArray)

            rospy.spin()
            rospy.loginfo("Stopped nav_goals_generator service")

        def map_callback(self,data):

            # get map data
            self.resolution = data.info.resolution
            self.width = data.info.width
            self.height = data.info.height
            self.origin = data.info.origin
            self.data = data.data

            self.map_min_x = self.origin.position.x
            self.map_max_x = self.origin.position.x + self.width * self.resolution
            self.map_min_y = self.origin.position.y
            self.map_max_y = self.origin.position.y + self.height * self.resolution


        def nav_goals_service(self,req):
            rospy.loginfo('Incoming service request: %s', req)
            
            # get arguments
            self.n = req.n
            self.inflation_radius = req.inflation_radius
            self.roi = req.roi

            # process arguments
            self.process_arguments()

            # generate response
            res = NavGoalsResponse()
            res.goals.header.frame_id = self.map_frame
            res.goals.poses = []

            # generate random goal poses
            while len(res.goals.poses) < self.n: 
                        
                cell_x = int(random.uniform(self.cell_min_x, self.cell_max_x))
                cell_y = int(random.uniform(self.cell_min_y, self.cell_max_y))

                pose = Pose()
                pose.position.x = cell_x * self.resolution + self.origin.position.x
                pose.position.y = cell_y * self.resolution + self.origin.position.y

                # if the point lies within ROI and is not in collision
                if self.in_roi(pose.position.x,pose.position.y) and not self.in_collision(cell_x,cell_y):

                    yaw = random.uniform(0, 2*math.pi)

                    q = list(tf.transformations.quaternion_about_axis(yaw, (0,0,1)))

                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]

                    #rospy.loginfo("pose x: %s, y: %s", pose.position.x, pose.position.y)
                    res.goals.poses.append(pose)

            self.pub.publish(res.goals)
            return res

        def process_arguments(self):
            # n must be positive
            if self.n < 0:
                self.n = 0

            # inflation radius must be positive     
            if self.inflation_radius < 0:
                self.inflation_raidus = 0.5

            self.inflated_footprint_size = int(self.inflation_radius / self.resolution) + 1
                
            # region of interest (roi) must lie inside the map boundaries
            # if roi is empty, the whole map is treated as roi by default
            if len(self.roi.points) == 0:
                self.bbox_min_x = self.map_min_x
                self.bbox_max_x = self.map_max_x
                self.bbox_min_y = self.map_min_y
                self.bbox_max_y = self.map_max_y
                rospy.loginfo('no ROI specified, full map is used.')

            else:
                # if the roi is outside the map, adjust to map boundaries
                # determine bbox of roi
                self.bbox_min_x = float('inf')
                self.bbox_max_x = float('-inf')
                self.bbox_min_y = float('inf')
                self.bbox_max_y = float('-inf')
                for p in self.roi.points:
                    if p.x < self.map_min_x:
                        p.x = self.map_min_x
                    if p.x > self.map_max_x:
                        p.x = self.map_max_x
                        
                    if p.x < self.bbox_min_x:
                        self.bbox_min_x = p.x
                    if p.x > self.bbox_max_x:
                        self.bbox_max_x = p.x
                                                
                    if p.y < self.map_min_y:
                        p.y = self.map_min_y
                    if p.y > self.map_max_y:
                        p.y = self.map_max_y
                        
                    if p.y < self.bbox_min_y:
                        self.bbox_min_y = p.y
                    if p.y > self.bbox_max_y:
                        self.bbox_max_y = p.y
                        
            # calculate bbox for cell array
            self.cell_min_x = int((self.bbox_min_x - self.origin.position.x) / self.resolution)
            self.cell_max_x = int((self.bbox_max_x - self.origin.position.x) / self.resolution)
            self.cell_min_y = int((self.bbox_min_y - self.origin.position.y) / self.resolution)
            self.cell_max_y = int((self.bbox_max_y - self.origin.position.y) / self.resolution)

            rospy.loginfo('ROI bounding box (meters): (%s,%s) (%s,%s)', \
                              self.bbox_min_x,self.bbox_min_y,self.bbox_max_x,self.bbox_max_y)

            rospy.loginfo('ROI bounding box (cells): (%s,%s) (%s,%s)', \
                          self.cell_min_x,self.cell_min_y,self.cell_max_x,self.cell_max_y)
                

        
        def cell(self, x,y):
                if x < 0 or y <0 or x >= self.width or y >= self.height:
                    #rospy.loginfo("out of bounds! x: %s, y: %s", x, y)
                    # return 'unknown' if out of bounds
                    return -1
                
                return self.data[x +  self.width * y]

        def in_roi(self,x,y):
            if (len(self.roi.points)==0):
                return True
            p = Point32(x,y,0)
            return is_inside(p, self.roi.points)
        
        def in_collision(self,x,y):
                x_min = x - self.inflated_footprint_size
                x_max = x + self.inflated_footprint_size                    
                y_min = y - self.inflated_footprint_size
                y_max = y + self.inflated_footprint_size                    
                for i in range(x_min,x_max):
                        for j in range(y_min,y_max):
                                if (self.cell(i,j) != 0):
                                        return True
                return False
                        

if __name__ == '__main__':
    NavGoalsGenerator()
