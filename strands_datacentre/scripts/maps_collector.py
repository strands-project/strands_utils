#!/usr/bin/env python

import roslib; roslib.load_manifest('strands_datacentre')
import rospy
import sys
import os
import collections
import json
from datetime import datetime
import glob
import re

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

        date_pat = re.compile('(\d+)-(\d+)-(\d+)-(\d+)-(\d+) (\S+)')

        for dirname, dirnames, filenames in os.walk('.'):
            dirnames.sort()
            for subdirname in dirnames:
                s_dir = str(os.path.join(dirname, subdirname))+'/*.pgm'
                pgmList = glob.glob(str(s_dir))
                pgmList.sort()
                for item in pgmList:
                    filename = item.rsplit('/')
                    n_fildict = int(item.count('/'))
                    f_name=filename[n_fildict]
                    f_name=f_name[:-4]
                    existing = self._maps_collection.find_one({"Filename":f_name})
                    if existing is None:
                        s_date=f_name + " BST"
                        if date_pat.match(s_date):
                            f_path = os.path.abspath(item)
                            date = datetime.strptime(s_date, '%d-%m-%y-%H-%M %Z')
                            rospy.loginfo("New map stored for %s"%f_name)
                            s_date=f_name + " BST"
                            date = datetime.strptime(s_date, '%d-%m-%y-%H-%M %Z')
                            self._maps_collection.insert({"Filename": f_name,"date": date,"path": f_path})

    def _on_node_shutdown(self):
        self._mongo_client.disconnect()

if __name__ == '__main__':
    server = MapsCollector()
