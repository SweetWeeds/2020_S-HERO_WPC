from collections import defaultdict
import os
import math
from heapq import heapify, heappush, heappop
import json
from pathlib import Path

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
            result += f'{v}: {str(self.graph[v])}, \n'
        return result

class Map:
    def __init__(self):
        path = Path(__file__).parent / "./map.json" # for relative path
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
        d = dict.fromkeys(graph.V, math.inf) # distance pair 
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

    """
        shortest_path
        input:
            s: start node ID
            t: target node ID
        output:
            path: path list
    """
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
            return f'unable to find shortest path staring from "{s}" to "{t}"'
        #print(path)
        #return f'{" > ".join(path)}'
        return path



if __name__ == "__main__":
    m = Map()
    print( m.shortest_path('B', 'E') )
