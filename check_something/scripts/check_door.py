#!/usr/bin/env python
import rospy
import actionlib
from check_something.checker import Checker
from strands_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from ros_mary_tts.msg import *

class DoorChecker(Checker):
	def __init__(self): 
		super( DoorChecker, self ).__init__('check_door', 'doorDetection', DoorCheckAction)
		self.speech_client = None

	def speak(self, text):
		if self.speech_client == None:
			action = 'speak'
			rospy.loginfo('waiting for %s server' % (action))
			self.speech_client = actionlib.SimpleActionClient(action, maryttsAction)
			self.speech_client.wait_for_server(rospy.Duration(10))

		if self.speech_client != None:
			goal = maryttsGoal(text)
			self.speech_client.send_goal_and_wait(goal, rospy.Duration(20))

		rospy.loginfo(text)

	def generate_report(self, state, result):	
		if state == GoalStatus.SUCCEEDED:
			if result.open:
				self.speak("I think the door is open")
			else:
				self.speak("I think the door is closed")
		else:	
			rospy.loginfo("Action interrupted or aborted")


if __name__ == '__main__':
    rospy.init_node("door_checker")
    checker = DoorChecker()  
    rospy.spin()

   