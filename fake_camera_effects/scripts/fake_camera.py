#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import fake_camera_effects.msg
import fake_camera_effects.utils as utils


class CameraEffects(object):
# create messages that are used to publish feedback/result

    def __init__(self, name):
        # Variables
        self._action_name = name

        self.photo = utils.Photo()

        # Starting server
        rospy.loginfo("Starting action server")
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            fake_camera_effects.msg.CameraEffectsAction,
            execute_cb=self.goalCallback,
            auto_start=False
        )
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("...done.")

    def goalCallback(self, goal):
        self.photo.photo()
        self._as.set_succeeded()

    def preemptCallback(self):
        self._as.set_preempted()

if __name__ == '__main__':
    rospy.init_node('camera_effects')
    CameraEffects(rospy.get_name())
    rospy.spin()
