#!/usr/bin/env python

import rospy
from strands_tweets.srv import *
import rospy
from twython import Twython, TwythonError

APP_KEY = 'vDcKFy7YrG2NNvjvH0BTVQ'
APP_SECRET = 'nDynC5hjYqBPW9xEEiULTG4FEHRTdCHTUVClHGu00'
OAUTH_TOKEN = '2192647862-uGT366zFzOQgz0W0XoqqG6YIR9HFltYCWZDD07I'
OAUTH_TOKEN_SECRET = '7ybaYLsFWhbTrRfFlxlIoxQX7SwiZbcmnidaOOgyc41nH'

from twython import TwythonStreamer

twitter = Twython(APP_KEY, APP_SECRET, OAUTH_TOKEN, OAUTH_TOKEN_SECRET)

class MyStreamer(TwythonStreamer):
    def on_success(self, data):
        if 'text' in data:
            received = data['text'].encode('utf-8')
            print received
            if '@SimLindaStrands' in received :
                request = received.replace("@SimLindaStrands", "");
                user = data['user']['screen_name']
                understood = False
                if 'Battery' in request:
                    answer = "@%s You asked for my Battery State, I will be able to answer soon" %user
                    print answer
                    understood = True
                if 'Position' in request:
                    answer = "@%s You asked for my Position, I will be able to answer soon" %user
                    print answer
                    understood = True
                if 'Status' in request:
                    answer = "@%s You asked for my Status, I will be able to answer soon" %user
                    print answer
                    understood = True
                if 'Who are you?' in request:
                    answer = "@%s I am Sim Linda Strands a Simualted Robot from the STRANDS family" %user
                    print answer
                    understood = True
                if not understood :
                    answer = "@%s I still don\'t know what you are talking about, but I will soon" %user
                    print answer
                twitter.update_status(status=answer)
                
        # Want to disconnect after the first result?
        # self.disconnect()

    def on_error(self, status_code, data):
        print status_code, data


if __name__ == '__main__':
    stream = MyStreamer(APP_KEY, APP_SECRET, OAUTH_TOKEN, OAUTH_TOKEN_SECRET)
    #stream.statuses.filter(track='twitter')
    stream.user()
    #stream.site(follow='twitter')
