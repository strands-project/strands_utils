#!/usr/bin/env python
import rospy
import actionlib
from check_something.checker import Checker
from strands_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus


class DoorChecker(Checker):
	def __init__(self): 
		super( DoorChecker, self ).__init__('check_door', 'doorDetection', DoorCheckAction)

	def generate_report(self, state, result):	
		if state == GoalStatus.SUCCEEDED:
			if result.open:
				rospy.loginfo("Door is open")
			else:
				rospy.loginfo("Door is not open")
		else:	
			rospy.loginfo("Action interrupted or aborted")


if __name__ == '__main__':
    rospy.init_node("door_checker")
    checker = DoorChecker()  
    rospy.spin()

   