#!/usr/bin/env python

import roslib; roslib.load_manifest('strands_tweets')
import sys

import rospy
from strands_tweets.srv import *

def tweet_client(text):
    rospy.wait_for_service('/strands_tweets/Tweet')
    try:
        tweet = rospy.ServiceProxy('/strands_tweets/Tweet', Tweet)
        resp1 = tweet(text,True)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print sys.argv
    text0 = " ".join(sys.argv[1:])
    #text0 = "Hello ROS World"
    print "Tweeting %s"%(text0)
    resp=tweet_client(text0)
    print resp
