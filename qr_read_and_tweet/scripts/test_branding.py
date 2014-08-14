#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import qr_read_and_tweet.msg
import cv2
#from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class brand_test_client(object):
    
    def __init__(self, filename) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        
        rospy.loginfo(" ... Initialising")
        
        self.client = actionlib.SimpleActionClient('/image_branding', qr_read_and_tweet.msg.ImageBrandingAction)
        self.client.wait_for_server()

        rospy.loginfo(" ... Init done")
    
        goal = qr_read_and_tweet.msg.ImageBrandingGoal()
    
        img = cv2.imread(filename)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        goal.photo = image_message
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  
        
        #print ps
        bridge2 = CvBridge()
        dst = bridge2.imgmsg_to_cv2(ps.branded_image, "bgr8")
        
        cv2.imshow('dst', dst)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    print 'Argument List:',str(sys.argv)
    if len(sys.argv) < 2 :
	sys.exit(2)
    rospy.init_node('brand_image_test')
    ps = brand_test_client(sys.argv[1])






