#!/usr/bin/env python
#-*- coding: utf-8 -*-
from collections import defaultdict
import os
import math
from heapq import heapify, heappush, heappop
import json
from pathlib import Path
import rospy
from std_msgs.msg import Empty, String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
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

past_line = None

past_path_buf = []

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

    # t: target node's name
    def shortest_path(self, t):
        s = self.getNearestNode()
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
            return None
        #print(path)
        #return f'{" > ".join(path)}'
        return path

    # Get Position
    def getPos(self, node_name):
        for m in self.map_data:
            if (node_name == m[0]):
                return m[0], m[1], m[2]
        return None, None, None
    def getNearestNode(self):
        data = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        shortest_dist = -1
        nearest_node = None
        for m in self.map_data:
            dist = math.sqrt((m[1]-x)**2 + (m[2]-y)**2)
            if dist < shortest_dist or shortest_dist == -1:
                shortest_dist = dist
                nearest_node = m[0]
        return nearest_node


# Function for map_router.launch
def launch():
    counter = 0
    def goal_callback(data):
        global counter

        goal_pos = data.data

        # Cancel Current Move Plan
        for i in range(counter):
            cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        
        counter = 0
        
        print("GOAL:{}".format(goal_pos))
        
        # Get Shortest Route
        path_buf = m.shortest_path(goal_pos)
        if(path_buf == None):
            print("NO ROUTE")
            exit()
        print(path_buf)

        # Write Route to csv file.
        with open(output_file_path, 'w') as file:
            for i, p in enumerate(path_buf):
                _, x, y = m.getPos(p)   # node, x, y
                if i+1 < len(path_buf) and p[0] == path_buf[i+1][0] and i != 0:
                    continue
                if(x == None or y == None):
                    print("Can't get position")
                    continue
                counter += 1
                file.write(str(x) + ',' + str(y) + ',' + '0.0,' + '0.0,' + '0.0,' + '1.0,' + '5.92660023892e-08' + '\n')
            rospy.loginfo('poses written to '+ output_file_path)
        
        rospy.sleep(1)

        # Start Move Plan
        start_journey_pub.publish(Empty())
    # end of goal_callback

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    # Map Instance
    print("Map Instance. MODE: Launch")
    m = Map()
    
    # init ROS node
    rospy.init_node('map_router')

    # Publishers
    path_ready_pub = rospy.Publisher('/path_ready', Empty, queue_size=10)
    path_reset_pub = rospy.Publisher('/path_reset', Empty, queue_size=10)
    start_journey_pub = rospy.Publisher('/start_journey', Empty, queue_size=1)
    waypoints_pub  = rospy.Publisher('/waypoints', PoseWithCovarianceStamped, queue_size=10)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    # Subscribers
    goal_sub = rospy.Subscriber('/goal', String, goal_callback)

    path_reset_pub.publish(Empty())

    rospy.spin()

# Function for stand alone execution
def main():
    counter = 0
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    # Map Instance
    print("Map Instance, MODE: main")
    m = Map()
    
    # init ROS node
    rospy.init_node('map_router')

    # Publishers
    path_ready_pub = rospy.Publisher('/path_ready', Empty, queue_size=10)
    path_reset_pub = rospy.Publisher('/path_reset', Empty, queue_size=10)
    start_journey_pub = rospy.Publisher('/start_journey', Empty, queue_size=1)
    waypoints_pub  = rospy.Publisher('/waypoints', PoseWithCovarianceStamped, queue_size=1)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    # Initialize Path Queue
    path_reset_pub.publish(Empty())
    while(True):
        # Set Goal Point
        print(input_text)
        goal_pos = raw_input()

        # Cancel Current Move Plan
        print("Plan Counter:{}".format(counter))
        for i in range(counter + 1):
            cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        
        counter = 0

        # Cancel Current Move Plan
        cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        cancel_pub.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        
        print("GOAL:{}".format(goal_pos))
        
        # Get Shortest Route
        path_buf = m.shortest_path(goal_pos)
        if(path_buf == None):
            print("NO ROUTE")
            exit()
        print(path_buf)

        # Write Route to csv file.
        with open(output_file_path, 'w') as file:
            for i, p in enumerate(path_buf):
                _, x, y = m.getPos(p)   # node, x, y
                if i+1 < len(path_buf) and p[0] == path_buf[i+1][0] and i != 0:
                    continue
                if(x == None or y == None):
                    print("Can't get position")
                    continue
                counter+=1
                file.write(str(x) + ',' + str(y) + ',' + '0.0,' + '0.0,' + '0.0,' + '1.0,' + '5.92660023892e-08' + '\n')
            rospy.loginfo('poses written to '+ output_file_path)
        
        rospy.sleep(1)

        # Start Move Plan
        start_journey_pub.publish(Empty())

if __name__ == "__main__":
    main()
