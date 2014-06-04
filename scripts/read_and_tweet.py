#! /usr/bin/env python

import rospy
import sys
from time import sleep
import actionlib
import strands_tweets.msg
import nhm.msg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client



class read_and_tweet(object):
    
    def __init__(self) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.msg_sub = rospy.Subscriber('/datamatrix/msg', String, self.datamatrix_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('strands_tweets', strands_tweets.msg.SendTweetAction)
        self.click = actionlib.SimpleActionClient('twitter_effects', nhm.msg.TwitterEffectsAction)
        
        self.rcnfclient = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')              
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")


    def datamatrix_callback(self, msg) :
        dmtx_msg = msg.data
        if dmtx_msg == '0' :
            self.msg_sub.unregister()
            config = self.rcnfclient.get_configuration()
            self.mvx = config['max_vel_x']
            params = { 'max_vel_x' : 0.0 }
            config = self.rcnfclient.update_configuration(params)
            self.img_subs = rospy.Subscriber('/head_xtion/rgb/image_color', Image, self.image_callback,  queue_size=1)

            

    def image_callback(self, msg) :
        clickgoal = nhm.msg.TwitterEffectsGoal()
        tweetgoal = strands_tweets.msg.SendTweetGoal()
        text = "Look who is here"
        print "tweeting %s" %text
    
        tweetgoal.text = text
        #navgoal.origin = orig
        tweetgoal.with_photo = True
        
        tweetgoal.photo = msg
        self.img_subs.unregister()
        
        
        self.click.send_goal(clickgoal)
        # Sends the goal to the action server.
        self.client.send_goal(tweetgoal)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  
        print ps
        sleep(10)
        self.msg_sub = rospy.Subscriber('/datamatrix/msg', String, self.datamatrix_callback, queue_size=1)
        params = { 'max_vel_x' : self.mvx }
        config = self.rcnfclient.update_configuration(params)
        

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('read_and_tweet')
    ps = read_and_tweet()
    rospy.spin()
