#!/usr/bin/env python

import rospy
import roslib
from sensor_msgs.msg import JointState
from sound_play.libsoundplay import SoundClient
from scitos_msgs.msg import HeadLightState


class Photo():
    def __init__(self):
        self.click = roslib.packages.get_pkg_dir('fake_camera_effects') + '/sounds/click.wav'
        self.head = Head()
        pub_topic = '/head/cmd_light_state'
        self.pub = rospy.Publisher(pub_topic, HeadLightState)

    def photo(self):
        self.head.closeEyes()
        self.soundhandle = SoundClient()
        rospy.sleep(rospy.Duration(1))
        self.flashLEDs()
        self.soundhandle.playWave(self.click)
        rospy.sleep(rospy.Duration(2))
        self.spinningLEDs()
        self.soundhandle.stopAll()
        self.head.openEyes()

    def spinningLEDs(self):
        lights = HeadLightState()
        lights.HeadLightInterval = 80
        lights.LEDState0 = 3
        lights.LEDState1 = 3
        lights.LEDState2 = 3
        lights.LEDState3 = 3
        lights.LEDState4 = 3
        lights.LEDState5 = 3
        lights.LEDState6 = 3
        lights.LEDState7 = 3
        lights.LEDPhase0 = 10
        lights.LEDPhase1 = 20
        lights.LEDPhase2 = 30
        lights.LEDPhase3 = 40
        lights.LEDPhase4 = 50
        lights.LEDPhase5 = 60
        lights.LEDPhase6 = 70
        lights.LEDPhase7 = 80
        lights.LEDAmplitude0 = 1.0
        lights.LEDAmplitude1 = 1.0
        lights.LEDAmplitude2 = 1.0
        lights.LEDAmplitude3 = 1.0
        lights.LEDAmplitude4 = 1.0
        lights.LEDAmplitude5 = 1.0
        lights.LEDAmplitude6 = 1.0
        lights.LEDAmplitude7 = 1.0
        if not rospy.is_shutdown():
            self.pub.publish(lights)

    def flashLEDs(self):
        lights = HeadLightState()
        lights.HeadLightInterval = 0
        lights.LEDState0 = 1
        lights.LEDState1 = 1
        lights.LEDState2 = 1
        lights.LEDState3 = 1
        lights.LEDState4 = 1
        lights.LEDState5 = 1
        lights.LEDState6 = 1
        lights.LEDState7 = 1
        lights.LEDPhase0 = 1
        lights.LEDPhase1 = 1
        lights.LEDPhase2 = 1
        lights.LEDPhase3 = 1
        lights.LEDPhase4 = 1
        lights.LEDPhase5 = 1
        lights.LEDPhase6 = 1
        lights.LEDPhase7 = 1
        lights.LEDAmplitude0 = 1.0
        lights.LEDAmplitude1 = 1.0
        lights.LEDAmplitude2 = 1.0
        lights.LEDAmplitude3 = 1.0
        lights.LEDAmplitude4 = 1.0
        lights.LEDAmplitude5 = 1.0
        lights.LEDAmplitude6 = 1.0
        lights.LEDAmplitude7 = 1.0
        if not rospy.is_shutdown():
            self.pub.publish(lights)


class Head():
    def __init__(self):
        self.pub = rospy.Publisher('/head/commanded_state', JointState)

    def resetHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [0, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def closeEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [0]
        self.pub.publish(self.head_command)

    def openEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [100]
        self.pub.publish(self.head_command)
