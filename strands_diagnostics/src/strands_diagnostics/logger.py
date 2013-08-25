# The implementation of the diagnostics logger 
from strands_datacentre.util import *
from strands_diagnostics.msg import StrandsDiagnosticMsg

import rospy

if check_for_pymongo():
    import pymongo
    have_pymongo = True
else:
    have_pymongo = False
    
import json

class DiagnosticsLogger(object):
    def __init__(self, node):
        self._node =  node
        self.ok = True
        
        # Check for mongodb
        have_db =  wait_for_mongo()
        if not have_db:
            self.ok = False
            return

        # check for pymongo
        if not have_pymongo:
            self.ok = False
            return
        
        self._diag_subscriber =  rospy.Subscriber("/strands_diagnostics",  StrandsDiagnosticMsg,  self._diagnostic_cb)

        host =  rospy.get_param("datacentre_host")
        port =  rospy.get_param("datacentre_port")
        self._mongoclient = pymongo.MongoClient(host, port)
        
        rospy.spin()

        
    def _diagnostic_cb(self, msg):
        collection = self._mongoclient[msg.package][msg.subpackage]
        doc =  {}
        doc["msg"] = json.loads(msg.json_data)
        now =  rospy.get_rostime()
        doc["stamp"] = {"secs": now.secs, "nasecs": now.nsecs,}
        collection.insert(doc)
        
        