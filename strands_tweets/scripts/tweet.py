#!/usr/bin/env python

from strands_tweets.srv import *
import rospy
from twython import Twython, TwythonError

class tweetsServer(object):
    def __init__(self):
        APP_KEY = rospy.get_param("/twitter/appKey")
        APP_SECRET = rospy.get_param("/twitter/appSecret")
        OAUTH_TOKEN = rospy.get_param("/twitter/oauthToken")
        OAUTH_TOKEN_SECRET = rospy.get_param("/twitter/oauthTokenSecret")
        self._twitter = Twython(APP_KEY, APP_SECRET, OAUTH_TOKEN, OAUTH_TOKEN_SECRET)
        rospy.init_node('strands_tweets')
        self._tweet_srv = rospy.Service("/strands_tweets/Tweet",Tweet,self._tweet_srv_cb)
        rospy.loginfo("Ready to Tweet ...")
        rospy.spin()

    def _tweet_srv_cb(self, req):
        nchar=len(req.text)
        rospy.loginfo("Tweeting %s ... (%d)", req.text, nchar)
        if nchar < 140:
            try:
                self._twitter.update_status(status=req.text)
                result=True
            except TwythonError as e:
                print e
                result=False
            return TweetResponse(result)
        else:
            if not req.force:
                result=False
                rospy.loginfo("You can't send tweet with more than 140 characters unless you set the force parameter to True, your tweet had %d characters", nchar)
                return TweetResponse(result)
            else:
                ntotaltweets=1+(nchar/130)
                rospy.loginfo("You will need %d tweets to publish this", ntotaltweets)
                line = req.text
                n = 130
                outtext=list([line[i:i+n] for i in range(0, len(line), n)])
                outtext.reverse()
                ntweets=ntotaltweets
                for w in outtext:
                    wo = "(%d/%d) %s" % (ntweets, ntotaltweets, w)
                    try:
                        self._twitter.update_status(status=wo)
                        result=True
                    except TwythonError as e:
                        print e
                        result=False
                    ntweets= ntweets-1
                return TweetResponse(result)

if __name__ == '__main__':
    server = tweetsServer()


