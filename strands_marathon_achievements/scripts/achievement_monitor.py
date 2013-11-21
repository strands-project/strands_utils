#! /usr/bin/env python

import roslib
import rospy
import pymongo
from  waypoint_patroller import log_util

        # self.start_time = None
        # self.start_mileage = 0
        # self.bumper_hits=[]
        # self.navigation_fails=[]
        # self.finish_time = None
        # self.start_mileage = 0
        # self.finish_mileage = 0
        # self.stamped_mileage=[]
        # self.stamped_battery=[]
        # self.stamped_charges=[]
        # self.waypoints_stamps=[]
        # self.active_point_name = ""

def wipe_achievement_records(achievements_db):
	achievements_db.remove()

def broadcase_achievement(type, current_value, target_value, achievement):
	print achievement

def handle_achievement(achievements_db, type, current_value, target_value, achievement):

	achievement_doc = {"type": type, "val": target_value}
	existing = achievements_db.find_one(achievement_doc)
	
	# if we haven't seen this before
	if existing == None:
		broadcase_achievement(type, current_value, target_value, achievement)
		achievements_db.insert(achievement_doc)




if __name__ == '__main__':
	rospy.init_node("achievement_monitor")
	host = rospy.get_param("datacentre_host") 
	port = rospy.get_param("datacentre_port")

	gen = log_util.StatGenerator(host, port) 
	client = pymongo.MongoClient(host, port) 
	db=client.autonomous_patrolling 
	achievements_db=db["achievements"]
	
	# dor debugging, wipe all records of previous achievements
	wipe_achievement_records(achievements_db)

	rate = rospy.Rate(0.1) # in hz -- summary is only updated every 5 minutes 
	while not rospy.is_shutdown():

		# load params
		achievement_list = rospy.get_param('/achievements')		

		# get latest episode
		latest = gen.get_episode(gen.get_latest_run_name())
		print latest.get_json_summary()

		# get duration from last entry
		# print latest.stamped_mileage[-1][1]

		for achievement_type in achievement_list:

			test_val = None

			# translate stat into target value type
			if achievement_type == 'run_duration':
				# hours
				test_val = (latest.stamped_mileage[-1][1].total_seconds()/60)/60
			elif achievement_type == 'distance':
				# km
				test_val = latest.stamped_mileage[-1][0]
			
			print test_val

			achievement_targets = achievement_list[achievement_type]

			for achievement_target in achievement_targets:

				# print achievement_target['val']				

				if test_val >= achievement_target['val']:
					handle_achievement(achievements_db, achievement_type, test_val, achievement_target['val'], achievement_target['achievement'])

		rate.sleep()
