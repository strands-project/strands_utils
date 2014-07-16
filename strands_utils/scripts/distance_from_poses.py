#!/usr/bin/env python

import rospy
import math 
import sys
import argparse

from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import PoseWithCovarianceStamped
from datetime import datetime


def mkdatetime(date_string):
    return datetime.strptime(date_string, '%d/%m/%y %H:%M')

if __name__ == "__main__":
    rospy.init_node("distance_from_poses")

    parser = argparse.ArgumentParser(description='Calculates the disance travelled from a collection of poses.')
    
    parser.add_argument('collection', metavar='C', nargs='?', default='people_tracks',
                   help='The message_store collection to take the poses from')

    parser.add_argument('start', metavar='S', type=mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=mkdatetime, nargs='?', default=datetime.utcnow(),
                   help='end datetime of query, defaults to now. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


    
    msg_store = MessageStoreProxy(collection=args.collection)
    meta_query = {}
    meta_query["inserted_at"] = {"$gte": args.start, "$lte" : args.end} 


    poses = msg_store.query(PoseWithCovarianceStamped._type, meta_query=meta_query)

    # sort by timestamps
    poses.sort(key=lambda x: (x[0].header.stamp.secs, x[0].header.stamp.nsecs))

    distance = 0
    for i in range(1, len(poses)): 
        distance += math.sqrt((poses[i][0].pose.pose.position.x - poses[i-1][0].pose.pose.position.x)**2 + (poses[i][0].pose.pose.position.y - poses[i-1][0].pose.pose.position.y)**2)

    print "Robot travelled %.2fkm" % (distance/1000)
    print "%s pose updates" % len(poses)
    print "First pose on %s " % poses[0][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')
    print " Last pose on %s " % poses[-1][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')



