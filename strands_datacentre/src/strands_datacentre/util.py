import rospy
from std_srvs.srv import Empty
import yaml

"""
Waits for the mongo server, as started through the
strands_datacentre/mongodb_server.py wrapper
Return True on success, False if server not even started.
"""
def wait_for_mongo():
    # Check that mongo is live, create connection
    try:
        rospy.wait_for_service("/datacentre/wait_ready",10)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Can't connect to MongoDB server. Make sure strands_datacentre/mongodb_server.py node is started.")
        return False
    wait = rospy.ServiceProxy('/datacentre/wait_ready', Empty)
    wait()
    return True

"""
Checks for required version of pymongo, returns True if found
"""
def check_for_pymongo():
    try:
        import pymongo
    except:
        print("ERROR!!!")
        print("Can't import pymongo, this is needed by strands_datacentre.")
        print("Make sure it is installed (sudo pip install pymongo)")
        return False

    if not "MongoClient" in dir(pymongo):
        print ("ERROR!!!")
        print("Can't import required version of pymongo. We need >= 2.3")
        print("Make sure it is installed (sudo pip install pymongo) not apt-get")
        return False
    
    return True

"""
Given a ROS msg and a dictionary of the right values, fill in the msg
"""
def _fill_msg(msg,dic):
    for i in dic:
        if isinstance(dic[i],dict):
            _fill_msg(getattr(msg,i),dic[i])
        else:
            setattr(msg,i,dic[i])
    

"""
Given a document in the database, return metadata and ROS message
"""
def document_to_msg(document, TYPE):
    meta = document["meta"]
    msg = TYPE()
    _fill_msg(msg,document["msg"])
    return meta,msg

"""
Store a ROS message into the DB
"""    
def store_message(collection, msg, meta):
    doc={}
    doc["meta"]=meta
    doc["msg"]=yaml.load(str(msg))  # TODO: this is inefficient, should improve
    collection.insert(doc)

"""
Store a ROS message sans meta data
"""
def store_message_no_meta(collection, msg):
    #print str(msg)
    doc=yaml.load(str(msg))  # TODO: this is inefficient, should improve
    collection.insert(doc)

"""
Load a ROS message from the DB, no meta
"""
def document_to_msg_no_meta(document, TYPE):
    msg = TYPE()
    _fill_msg(msg, document)
    return meta, msg
