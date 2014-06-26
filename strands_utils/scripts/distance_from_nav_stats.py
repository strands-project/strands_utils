#!/usr/bin/env python

import rospy
import math 
import sys
import argparse

from ros_datacentre.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import NavStatistics, TopologicalNode
from datetime import datetime, timedelta


def mkdatetime(date_string):
    return datetime.strptime(date_string, '%d/%m/%y %H:%M')

if __name__ == "__main__":
    rospy.init_node("distance_from_nav_stats")

    parser = argparse.ArgumentParser(description='Calculates the disance travelled from a collection of stats.')
    
    parser.add_argument('collection', metavar='C', nargs='?', default='message_store',
                   help='The message_store collection to take the stats from')

    parser.add_argument('start', metavar='S', type=mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of query, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    parser.add_argument('end', metavar='E', type=mkdatetime, nargs='?', default=datetime.utcnow(),
                   help='end datetime of query, defaults to now. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')

    
    args = parser.parse_args()


    
    msg_store = MessageStoreProxy(collection=args.collection)
    meta_query = {}
    meta_query["inserted_at"] = {"$gte": args.start, "$lte" : args.end} 


    stats = msg_store.query(NavStatistics._type, meta_query=meta_query)

    # sort by timestamps
    stats.sort(key=lambda x: (x[1]['inserted_at']))

    if len(stats) == 0:
        print 'No stats found'
    else:    

        top_map_name = stats[0][0].topological_map
        top_map_msg_store = MessageStoreProxy(collection='topological_maps')

        nodes = top_map_msg_store.query(TopologicalNode._type, {"pointset" : top_map_name})

        if len(nodes) == 0:
            print 'No nodes found for top map %s' % top_map_name
        else:            

            node_positions = {}
            for node, meta in nodes:
                node_positions[node.name] = node.pose.position


            distance = 0
            duration_secs = 0
            for nav_stat, meta in stats:
                duration_secs +=  nav_stat.operation_time
                distance += math.sqrt((node_positions[nav_stat.origin].x - node_positions[nav_stat.final_node].x)**2 + (node_positions[nav_stat.origin].y - node_positions[nav_stat.final_node].y)**2)

            print "Robot travelled %.2fkm using %s travel time" % (distance / 1000, timedelta(seconds=duration_secs))
            print "%s stat updates" % len(stats)
            print "First stat on %s " % stats[0][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')
            print " Last stat on %s " % stats[-1][1]["inserted_at"].strftime('%d/%m/%y %H:%M:%S')



