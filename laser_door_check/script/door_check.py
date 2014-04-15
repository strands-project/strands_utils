#! /usr/bin/env python

import math
import rospy
import actionlib
import laser_door_check.msg

from time import sleep
from sensor_msgs.msg import LaserScan

class DoorCheck(object):
    # create messages that are used to publish feedback/result
    _feedback = laser_door_check.msg.DoorCheckFeedback()
    _result   = laser_door_check.msg.DoorCheckResult()
    _laser_received=False

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo('Starting Action Server')
        self._as = actionlib.SimpleActionServer(self._action_name, laser_door_check.msg.DoorCheckAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('Done')
    
    def execute_cb(self, goal):
        # publish info to the console for the user
        self._laser_received=False
        if goal.distance <= 0 :
            rospy.loginfo('distance has to be greater than 0')
            self._result.open=False
            self._as.set_succeeded(self._result)
        else :
            rospy.loginfo('%s: Executing, checking door at %i m' % (self._action_name, goal.distance))
            self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
            # start executing the action
            timeout=0
            while (not self._laser_received) and timeout < 100:
                sleep(0.1)
                timeout=timeout+1
            
            if self._laser_received :
                increment = (self.angle_max - self.angle_min)/ float(len(self._laser_data))
                start_angle = math.atan(0.4/goal.distance)
                start_sample = int(len(self._laser_data)-math.floor((start_angle-self.angle_min) / increment))
                final_sample = int(len(self._laser_data)-math.floor((self.angle_max-start_angle) / increment))
                i=start_sample
                self._result.open=True            
                while i<= final_sample and self._result.open :
                    if self._laser_data[i] < goal.distance :
                        self._result.open = False
                    i= i+1
                #print self._result
                #print self._laser_data
                self._as.set_succeeded(self._result)
            else :
                print "no laser data"
                self._result.open=False
                self._as.set_succeeded(self._result)
    
    def laser_callback(self, data):
        #rospy.loginfo(rospy.get_name() + ": I heard %s" % data.charging)
        #print ('%f to %f at %f/%d' %(data.angle_min, data.angle_max, data.angle_increment, len(data.ranges)))
        self.angle_min = data.angle_min
        self.angle_max= data.angle_max
        #selfdata.angle_increment, len(data.ranges)
        self._laser_data=data.ranges
        self._laser_received=True
        self.laser_sub.unregister()
            
if __name__ == '__main__':
  rospy.init_node('door_check')
  DoorCheck(rospy.get_name())
  rospy.spin()

