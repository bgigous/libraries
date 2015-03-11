#pragma once
#include "search_algorithms.h"
#include <limits>
using namespace std;

vector<vector<int> > Dijkstra(double * graph_first, int graph_size, int source){
    /*------------------------------------------------------------------------------*
    |   Inputs: pointer to first element of a square matrix of size graph_size, and |
    |           the starting node                                                   |
    |   Output: 2D vector of paths, where paths[x] returns a sequence of distance-  |
    |           optimal paths, and paths[x][y] returns y'th step on the path to x   |
    |   *** Pseudocode from: http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm    |
    *------------------------------------------------------------------------------*/

    // Initializations

	vector<vector<double> > Graph(graph_size,vector<double>(graph_size)); // Square matrix of distances between nodes: inf if unconnected
	for (int i=0; i<graph_size; i++){
        for (int j=0; j<graph_size; j++){
            Graph[i][j]=graph_first[i*graph_size+j];
        }
    }
	vector<double> dist(graph_size); // Distance between nodes, initially set to infinity (unreachable)
    vector<int> previous(graph_size);   // Previous node in optimal path from source
    vector<vector<int> > paths; // Path from start node to end node

    for (int i=0; i<graph_size; i++){
        dist[i] = numeric_limits<double>::infinity();
        previous[i] = -1;
    }
    dist[source] = 0;
    set<pair<double, int> > Q;  // Set of all nodes in graph, sorted by distance to starting point
    for (int i=0; i<graph_size; i++){
        Q.insert(make_pair(dist[i], i));
    }
    while (!Q.empty()){
        int u = Q.begin()->second;  // Grabs node with the smallest distance in dist[]
        if (dist[u]==numeric_limits<double>::infinity()) {
            break; // All remaining vertices inaccessible from source
        }
        Q.erase(Q.begin());         // Erases node with smallest distance in dist

        // Iterates through neighbors of 'u'
        set<pair<double, int> >::iterator it;
        for (it=Q.begin(); it!=Q.end(); ++it){
            int v = it->second;
            double alt = dist[u] + Graph[u][v];
            if (alt<dist[v]){
                dist[v] = alt;
                previous[v] = u;
                Q.erase(it), Q.insert(make_pair(dist[v],v));
            }
        }
    }

    for (int i=0; i<graph_size; i++){
        vector<int> S;      // Empty sequence (single path)
        int u = i;
        while(previous[u]!=-1){
            S.push_back(u);
            u = previous[u];
        }
        vector<int> rS;
        for (unsigned j=0; j<S.size(); j++){
            rS.push_back(S[S.size()-j-1]);
        }
        paths.push_back(rS);
    }
    return paths;
}



vector<int> Dijkstra(double * graph_first, int graph_size, int source, int target){
    /*------------------------------------------------------------------------------*
    |   Inputs: pointer to first element of a square matrix of size graph_size,     |
    |           the starting node, and the destination node                         |
    |   Output: vector of hops to get to the destination node                       |
    |   *** Pseudocode from: http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm    |
    *------------------------------------------------------------------------------*/

    // Initializations
    vector<vector<double> > Graph(graph_size,vector<double>(graph_size)); // Square graph: matrix of distances between nodes
    for (int i=0; i<graph_size; i++){
        for (int j=0; j<graph_size; j++){
            Graph[i][j]=graph_first[i*graph_size+j];
        }
    }

    vector<double> dist(graph_size);    // Distance between nodes, initially set to infinity (unreachable)
    vector<int> previous(graph_size);   // Previous node in optimal path from source

    for (int i=0; i<graph_size; i++){
        dist[i] = numeric_limits<double>::infinity();
        previous[i] = -1;
    }
    dist[source] = 0;
    set<pair<double, int> > Q;      // Set of all nodes in graph, sorted by distance to starting point
    for (int i=0; i<graph_size; i++){
        Q.insert(make_pair(dist[i], i));
    }
    while (!Q.empty()){
        int u = Q.begin()->second;  // Grabs node with the smallest distance in dist[]
        if (dist[u]==numeric_limits<double>::infinity()) {
            break;                  // All remaining vertices inaccessible from source
        }
        Q.erase(Q.begin());         // Erases node with smallest distance in dist

        // Iterates through neighbors of 'u'
        set<pair<double, int> >::iterator it;
        for (it=Q.begin(); it!=Q.end(); ++it){
            int v = it->second;
            double alt = dist[u] + Graph[u][v];
            if (alt<dist[v]){
                dist[v] = alt;
                previous[v] = u;
                Q.erase(it), Q.insert(make_pair(dist[v],v));
            }
        }
    }

    vector<int> S;      // Empty sequence (single path)
    int u=target;
    while(previous[u]!=-1){
        S.push_back(u);
        u = previous[u];
    }
    vector<int> rS;     // Reverse S
    for (unsigned i=0; i<S.size(); i++){
        rS.push_back(S[S.size()-i-1]);
    }
    return rS;
}
