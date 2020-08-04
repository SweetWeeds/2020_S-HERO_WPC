#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <list>
#include <limits>
#include <set>
#include <utility>
#include <algorithm>
#include <iterator>
#include "json/json.h"

#define MAX_MAP_LEN 100

using namespace std;

typedef int vertex_t;
typedef double weight_t;

// Constant Infinity
const weight_t max_weight = numeric_limits<double>::infinity();

// Relationship between vertexes.
struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};
 
typedef vector<vector<neighbor> > adjacency_list_t;

void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          vector<weight_t> &min_distance,
                          vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    set<pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(make_pair(min_distance[source], source));
 
    while (!vertex_queue.empty()) 
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());
 
        // Visit each edge exiting u
	const vector<neighbor> &neighbors = adjacency_list[u];
        for (vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(make_pair(min_distance[v], v));
 
	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(make_pair(min_distance[v], v));
 
	    }
 
        }
    }
}
 
 
list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const vector<vertex_t> &previous)
{
    list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}
 
 
int main()
{
    Json::Value root;
    Json::Reader reader;
    ifstream json("map.json", ifstream::binary);
    reader.parse(json, root);

    adjacency_list_t adjacency_list(MAX_MAP_LEN);

    for (auto& value:root["integer_key"]) {
        value:asInt()<<
    }
    /**
    adjacency_list[0].push_back(neighbor(1, 4));
    adjacency_list[0].push_back(neighbor(7, 8));
 
    adjacency_list[1].push_back(neighbor(0, 4));
    adjacency_list[1].push_back(neighbor(2, 8));
    adjacency_list[1].push_back(neighbor(7, 11));
 
    adjacency_list[2].push_back(neighbor(1, 8));
    adjacency_list[2].push_back(neighbor(3, 7));
    adjacency_list[2].push_back(neighbor(5, 4));
    adjacency_list[2].push_back(neighbor(8, 2));
 
    adjacency_list[3].push_back(neighbor(3, 7));
    adjacency_list[3].push_back(neighbor(4, 9));
    adjacency_list[3].push_back(neighbor(5, 14));
 
 
    adjacency_list[4].push_back(neighbor(3, 9));
    adjacency_list[4].push_back(neighbor(5, 10));
    
    adjacency_list[5].push_back(neighbor(2, 4));
    adjacency_list[5].push_back(neighbor(3, 14));
    adjacency_list[5].push_back(neighbor(4, 10));
    adjacency_list[5].push_back(neighbor(6, 2));
 
    adjacency_list[6].push_back(neighbor(5, 2));
    adjacency_list[6].push_back(neighbor(7, 1));
    adjacency_list[6].push_back(neighbor(8, 6));
 
    
    adjacency_list[7].push_back(neighbor(0, 9));
    adjacency_list[7].push_back(neighbor(1, 9));
    adjacency_list[7].push_back(neighbor(6, 9));
    adjacency_list[7].push_back(neighbor(8, 9));
    
 
    adjacency_list[8].push_back(neighbor(2, 2));
    adjacency_list[8].push_back(neighbor(6, 6));
    adjacency_list[8].push_back(neighbor(7, 7));
    **/
    // vector<weight_t> min_distance(MAX_MAP_LEN);
    vector<vector<weight_t>> min_distance(MAX_MAP_LEN);
    vector<vertex_t> previous;
    DijkstraComputePaths(0, adjacency_list, min_distance[0], previous);
    cout << "Distance from 0 to 5: " << min_distance[0][5] << endl;
    list<vertex_t> path = DijkstraGetShortestPathTo(5, previous);
    cout << "Path : ";
    copy(path.begin(), path.end(), ostream_iterator<vertex_t>(cout, " "));
    cout << endl;
    return 0;
}