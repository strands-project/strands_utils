#!/usr/bin/env python
import roslib; roslib.load_manifest('nav_goals_generator')
import rospy

from nav_goals_msgs.srv import *

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

import random
import math
import numpy

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

def vector_norm(data, axis=None, out=None):
    data = numpy.array(data, dtype=numpy.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(numpy.dot(data, data))
        data *= data
        out = numpy.atleast_1d(numpy.sum(data, axis=axis))
        numpy.sqrt(out, out)
        return out
    else:
        data *= data
        numpy.sum(data, axis=axis, out=out)
        numpy.sqrt(out, out)


def quaternion_about_axis(angle, axis):
    q = numpy.array([0.0, axis[0], axis[1], axis[2]])
    qlen = vector_norm(q)
    if qlen > _EPS:
        q *= math.sin(angle/2.0) / qlen
    q[0] = math.cos(angle/2.0)
    return q


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

            # for visualizing nav goals in RVIZ
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

        def nav_goals_service(self,req):

            self.n = req.n
            self.inflation_radius = req.inflation_radius
            
            if self.n < 0:
                self.n = 0

            if self.inflation_radius < 0:
                self.inflation_raidus = 0.5
                
            self.inflated_footprint_size = int(self.inflation_radius / self.resolution) + 1
                
            res = NavGoalsResponse()
            res.goals.header.frame_id = self.map_frame
            res.goals.poses = []

            # generate random goal poses
            while len(res.goals.poses) < self.n: 
                        
                x_cell = int(random.uniform(0, self.width))
                y_cell = int(random.uniform(0, self.height))
                
                if not self.in_collision(x_cell,y_cell):
                    pose = Pose()
                    
                    pose.position.x = x_cell * self.resolution + self.origin.position.x
                    pose.position.y = y_cell * self.resolution + self.origin.position.y

                    yaw = random.uniform(0, 2*math.pi)
                    q = list(quaternion_about_axis(yaw, [0,0,1]))

                    pose.orientation.w = q[0]
                    pose.orientation.x = q[1]
                    pose.orientation.y = q[2]
                    pose.orientation.z = q[3]

                    #rospy.loginfo("pose x: %s, y: %s", pose.position.x, pose.position.y)
                    res.goals.poses.append(pose)

            self.pub.publish(res.goals)
            return res

        def cell(self, x,y):
                if x < 0 or y <0 or x >= self.width or y >= self.height:
                    #rospy.logerr("Out of bounds! x: %s, y: %s", x, y)
                    # return 'unknown' if out of bounds
                    return -1
                
                return self.data[x +  self.width * y]

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
