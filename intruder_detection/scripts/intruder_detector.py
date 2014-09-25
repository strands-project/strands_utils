#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import intruder_detection.msg
import strands_pedestrian_tracking.msg


class IntruderDetection(object):
# create messages that are used to publish feedback/result
    _result = intruder_detection.msg.IntruderDetectionResult()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.counter = 0

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            intruder_detection.msg.IntruderDetectionAction,
            execute_cb=None,
            auto_start=False
        )
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        rospy.Subscriber(
            "/pedestrian_tracking/pedestrian_array",
            strands_pedestrian_tracking.msg.PedestrianTrackingArray,
            self.callback
        )

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        self.counter = 0

    def preemptCallback(self):
        self.counter = 1000
        self._as.set_preempted()

    def callback(self, msg):
        if not self._as.is_active():
            return

        if len(msg.pedestrians) > 0:
            self._result.found = True
            self._as.set_succeeded(self._result)
        else:
            self.counter += 1

        if self.counter > 40:
            self._result.found = False
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('intruder_detection')
    IntruderDetection(rospy.get_name())
    rospy.spin()
