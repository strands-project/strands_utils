#! /usr/bin/env python

import rospy
import sys
from time import sleep
import actionlib
import strands_tweets.msg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class read_and_tweet(object):
    
    def __init__(self) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        rospy.Subscriber('/datamatrix/msg', String, self.datamatrix_callback)
        self.client = actionlib.SimpleActionClient('strands_tweets', strands_tweets.msg.SendTweetAction)
              
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")


    def datamatrix_callback(self, msg) :
        dmtx_msg = msg.data
        if dmtx_msg == '0' :
            self.img_subs = rospy.Subscriber('/head_xtion/rgb/image_color', Image, self.image_Callback)
            

    def image_callback(self, msg) :
            
        tweetgoal = strands_tweets.msg.SendTweetGoal()
        text = "Look who is here"
        print "tweeting %s" %text
    
        tweetgoal.text = text
        #navgoal.origin = orig
        tweetgoal.with_photo = True
        
        tweetgoal.photo = msg
        self.img_subs.unregister
        
        # Sends the goal to the action server.
        self.client.send_goal(tweetgoal)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  
        print ps
        sleep(10)
        

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('read_and_tweet')
    ps = read_and_tweet()
