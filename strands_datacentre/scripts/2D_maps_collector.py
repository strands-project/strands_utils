#!/usr/bin/env python

import roslib; roslib.load_manifest('strands_datacentre')
import rospy
import sys
import os
import collections
import json
from datetime import datetime
import glob

import strands_datacentre.util

from strands_datacentre.srv import *
from std_srvs.srv import *
import rosparam

if not strands_datacentre.util.check_for_pymongo():
    sys.exit(1)

import pymongo

class MapsCollector(object):
    def __init__(self):
        rospy.init_node("maps_collector")
        rospy.on_shutdown(self._on_node_shutdown)

        if not strands_datacentre.util.wait_for_mongo():
            sys.exit(1)
        
        self._mongo_client = pymongo.MongoClient(rospy.get_param("datacentre_host","localhost"),
                                                 int(rospy.get_param("datacentre_port")))
        self._mongo_db = self._mongo_client.maps_database
        self._maps_collection = self._mongo_db.maps2d

		# Copy the defaults into the DB if not there already
        pgmList = glob.glob("./*.pgm")
        pgmList.sort()

        for item in pgmList:
            f_name=item[2:-4]
            existing = defaults_collection.find_one({"Filename":f_name})
            if existing is None:
                f_path = os.path.abspath(item)
                rospy.loginfo("New map stored for %s"%f_name)
                s_date=filename + " BST"
                date = datetime.strptime(s_date, '%d-%m-%y-%H-%M %Z')
                defaults_collection.insert({"Filename": f_name, 
                                            "date": date, 
                                            "path": f_path})
if __name__ == '__main__':
    server = MapsCollector()
