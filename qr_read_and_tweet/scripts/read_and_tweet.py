#! /usr/bin/env python

import rospy
import sys
from time import sleep
import actionlib
import json
from random import randint
from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
import strands_tweets.msg
import qr_read_and_tweet.msg
from fake_camera_effects.msg import CameraEffectsAction, CameraEffectsGoal
from strands_webserver.msg import WebloaderTmpPageAction, WebloaderTmpPageGoal
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import nhm.srv


class read_and_tweet(object):

    def __init__(self, behaviour):

        rospy.on_shutdown(self._on_node_shutdown)
        self.b_config = self.loadConfig(behaviour)
        self._killall_timers = False

        self.msg_sub = rospy.Subscriber('/datamatrix/msg', String, self.datamatrix_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('strands_tweets', strands_tweets.msg.SendTweetAction)
        self.click = actionlib.SimpleActionClient('camera_effects', CameraEffectsAction)
        self.brandclient = actionlib.SimpleActionClient('/image_branding', qr_read_and_tweet.msg.ImageBrandingAction)
        self.tmppageClient = actionlib.SimpleActionClient('/webloader/tmppage', WebloaderTmpPageAction)

        self.tw_pub = rospy.Publisher('/nhm/twitter/message', String, latch=True)
        self.rcnfclient = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')

        self.client.wait_for_server()
        self.brandclient.wait_for_server()
        self.click.wait_for_server()
        self.tmppageClient.wait_for_server()
        rospy.loginfo(" ... Init done")

    def datamatrix_callback(self, msg):
        dmtx_msg = msg.data
        if dmtx_msg == '0':
            self.msg_sub.unregister()
            self.counter = 5
            t = Timer(1.0, self.time_callback)
            t.start()
            config = self.rcnfclient.get_configuration()
            self.mvx = config['max_vel_x']
            self.mtv = config['max_trans_vel']
            self.mrv = config['max_rot_vel']
            self.mix = config['min_vel_x']
            self.mit = config['min_trans_vel']
            self.mir = config['min_rot_vel']
            params = {'max_vel_x' : 0.0, 'max_trans_vel':0.0, 'max_rot_vel':0.0, 'min_vel_x':0.0,'min_trans_vel':0.0, 'min_rot_vel':0.0 }
            config = self.rcnfclient.update_configuration(params)
            self.img_subs = rospy.Subscriber('/head_xtion/rgb/image_color', Image, self.image_callback,  queue_size=1)
        elif dmtx_msg == '1':
            clickgoal = CameraEffectsGoal()
            rospy.wait_for_service('/nhm/smach/wait')
            try:
                s = rospy.ServiceProxy('/nhm/smach/wait', nhm.srv.Wait)
                s(True)
                self.click.send_goal(clickgoal)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        elif dmtx_msg == '2':
            clickgoal = CameraEffectsGoal()
            rospy.wait_for_service('/nhm/smach/wait')
            try:
                s = rospy.ServiceProxy('/nhm/smach/wait', nhm.srv.Wait)
                s(False)
                self.click.send_goal(clickgoal)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        elif dmtx_msg == '5':
            clickgoal = CameraEffectsGoal()
            rospy.wait_for_service('/nhm/smach/patrol')
            try:
                s = rospy.ServiceProxy('/nhm/smach/patrol', nhm.srv.Patrol)
                s(True)
                self.click.send_goal(clickgoal)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e


    def image_callback(self, msg) :
        clickgoal = CameraEffectsGoal()
        tweetgoal = strands_tweets.msg.SendTweetGoal()
        brandgoal = qr_read_and_tweet.msg.ImageBrandingGoal()

        self.img_subs.unregister()

        tweets = self.b_config["tweets"]["card"]
        text = tweets[randint(0, len(tweets)-1)]
        #text = "Look who is here"
        print "tweeting %s" %text


        brandgoal.photo = msg
        self.brandclient.send_goal(brandgoal)
        self.brandclient.wait_for_result()
        br_ph = self.brandclient.get_result()


        tweettext=String()
        tweettext.data=text
        self.tw_pub.publish(tweettext)


        tweetgoal.text = text
        tweetgoal.with_photo = True
        tweetgoal.photo = br_ph.branded_image


        self.click.send_goal(clickgoal)
        self.client.send_goal(tweetgoal)

        self.client.wait_for_result()
        ps = self.client.get_result()
        print ps

        self.loadTmpPage(True, 'nhm-twitter.html', 15)

        sleep(3)
        self.msg_sub = rospy.Subscriber('/datamatrix/msg', String, self.datamatrix_callback, queue_size=1)


    def loadConfig(self, data_set) :
        msg_store = MessageStoreProxy(collection="hri_behaviours")
        query_meta = {}
        query_meta["nhm"] = data_set
        if len(msg_store.query(String._type, {}, query_meta)) == 0 :
            rospy.logerr("Desired dialogue options '"+data_set+"' not in datacentre.")
            raise Exception("Can't find data in datacentre.")
        else:
            message = msg_store.query(String._type, {}, query_meta)
            return json.loads(message[0][0].data)



    # Create page with a timeout in seconds
    def loadTmpPage(self, relative, page, timeout):
        goal = WebloaderTmpPageGoal()
        goal.relative = relative
        goal.page = page
        goal.timeout = timeout
        self.tmppageClient.send_goal(goal)


    def time_callback(self):
        if self.counter > 0 and not self._killall_timers :
            self.counter -= 1
            t = Timer(1.0, self.time_callback)
            t.start()
        else :
            params = { 'max_vel_x':self.mvx, 'max_trans_vel':self.mtv, 'max_rot_vel':self.mrv, 'min_vel_x':self.mix, 'min_trans_vel':self.mit, 'min_rot_vel':self.mir }
            config = self.rcnfclient.update_configuration(params)
            self.counter = 0

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        self._killall_timers=True
        #sleep(2)



if __name__ == '__main__':
    if len(sys.argv) < 2 :
        print "usage: insert_map input_file.txt dataset_name map_name"
        sys.exit(2)
    rospy.init_node('read_and_tweet')
    ps = read_and_tweet(sys.argv[1])
    rospy.spin()
