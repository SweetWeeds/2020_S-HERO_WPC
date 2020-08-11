#!/usr/bin/env python
#-*- coding: utf-8 -*-
from collections import defaultdict
import os
import math
from heapq import heapify, heappush, heappop
import json
from pathlib import Path
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospkg

import sys, select
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
waypoints = []

#from std_srvs.srv import Trigger, TriggerResponse
#from geometry_msgs.msg import PointStamped

#
# map.json
# [[node_list], [node_coordinate], [node_connection]]
# ex.
# [
#     [
#         "A1", "A2", "A3", ... , "node_name"
#     ],
#     [
#         ["A1", 3, 5], ["A2", 5, 5], ... ,["node_name", x, y]
#     ],
#     [
#         ["A1", "A2"], ["A2", "A3"], ... , ["node_name1", "node_name2"]
#     ]
# ]
#

input_text = """
Please Input Waypoint.
Example: A5
"""

e = """
Communications Failed
"""

# utility: priority queue
class Pq:
    def __init__(self):
        self.queue = []
        
    def __str__(self):
        return str(self.queue)
        
    def insert(self, item):
        heappush(self.queue, item)
    
    def extract_min(self):
        return heappop(self.queue)[1]
    
    def update_priority(self, key, priority):
        for v in self.queue:
            if v[1] == key:
                v[0] = priority
        heapify(self.queue)
    
    def empty(self):
        return len(self.queue) == 0

# utility: Graph
class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = defaultdict(lambda: [])
    
    def add_edge(self, v, u, w):
        self.graph[v].append((u, w))
        
    def __str__(self):
        result = ''
        for v in self.V:
            result += '{}: {}, \n'.format(v, str(self.graph[v]))
            #result += f'{v}: {str(self.graph[v])}, \n'
        return result

class Map:
    def __init__(self):
        path = Path(__file__).parent / "./map.json" # for relative path
        self.graph_data = None
        self.map_data = None
        self.connection_data = None
        self.g = None
        # Load json file (./map.json)
        with path.open() as json_stream:
            self.graph_data, self.map_data, self.connection_data = json.load(json_stream)
            self.g = Graph(self.graph_data)
            self.calcDistance()
            for n1, n2, w in self.distance_data:
                self.g.add_edge(n1, n2, w)

    def isConnected(self, node1, node2):
        for cd in self.connection_data:
            if ((cd[0] == node1 and cd[1] == node2) or (cd[1] == node1 and cd[0] == node2)):
                return True
        return False

    def calcDistance(self):
        self.distance_data = list()
        for idx1, data1 in enumerate(self.map_data):
            for idx2, data2 in enumerate(self.map_data):
                if (idx1 == idx2):
                    continue
                if (self.isConnected(data1[0], data2[0])):
                    self.distance_data.append([data1[0], data2[0], math.sqrt((data1[1]-data2[1])**2 + (data1[2]-data2[2])**2)])
        return self.distance_data

    def dijkstra(self, graph, s):
        Q = Pq() # priority queue of vertices
                # [ [distance, vertex], ... ] 
        #d = dict.fromkeys(graph.V, math.inf) # distance pair 
        d = dict.fromkeys(graph.V, float('inf'))
                                            # will have default value of Infinity
        pi = dict.fromkeys(graph.V, None) # map of parent vertex
                                        # useful for finding shortest path	
        
        # initialize
        d[s] = 0
        
        # update priority if prior path has larger distance
        def relax(u, v, w):
            if d[v] > d[u] + w:
                d[v] = d[u] + w
                Q.update_priority(v, d[v])
                pi[v] = u
        
        # initialize queue
        for v in graph.V:
            Q.insert([d[v], v])
        
        while not Q.empty():
            u = Q.extract_min()
            for v, w in graph.graph[u]:
                relax(u, v, w)
            
        return d, pi

    # s: start node's name
    # t: target node's name
    def shortest_path(self, s, t):
        d, pi = self.dijkstra(self.g, s)
        path = [t]
        current = t
        
        # if parent pointer is None,
        # then it's the source vertex
        while pi[current]:
            path.insert(0, pi[current])
            # set current to parent
            current = pi[current]
            
        if s not in path:
            return 'unable to find shortest path staring from "{}" to "{}"'.format(s, t)
        #print(path)
        #return f'{" > ".join(path)}'
        return path

    # Get Position
    def getPos(self, node_name):
        for m in self.map_data:
            if (node_name == m[0]):
                return m[1], m[2]
        return None, None

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    # Map Instance
    m = Map()
    
    # init ROS node
    rospy.init_node('map_router')

    # Publishers
    path_ready_pub = rospy.Publisher('/path_ready', std_msgs.Empty, queue_size=10)
    path_reset_pub = rospy.Publisher('/path_reset', std_msgs.Empty, queue_size=10)
    waypoints_pub  = rospy.Publisher('/waypoints', geometry_msgs.PoseWithCovarianceStamped, queue_size=10)
    
    current_pos = "A0"

    try:
        while(True):
            path_reset_pub.publish()
            goal_pos = input(input_text)

            path_buf = m.shortest_path(current_pos, goal_pos)
            for p in path_buf:
                pose_buf = PoseWithCovarianceStamped()
                pose_buf.header.frame_id = "map"
                pose_buf.pose.pose.position.x, pose_buf.pose.pose.position.y = getPos(p)
                waypoints_pub.publish(pose_buf)

            current_pos = goal_pos
            path_ready_pub.publish()
    except:
        print()