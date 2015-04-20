#pragma once

// Boost includes
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>

// STL includes
#include <time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt

// library includes
#include "../Math/easymath.h"

using namespace boost;
using namespace std;
using namespace easymath;

typedef float cost;

// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap l, Vertex goal): 
		m_location(l), 
		m_goal(goal) 
	{};
	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return ::sqrt(dx * dx + dy * dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if(u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};


#pragma once
class AStar_easy
{
public:
	typedef adjacency_list<listS, vecS, undirectedS, no_property,
		property<edge_weight_t, cost> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef mygraph_t::vertex_iterator vertex_iterator;
	typedef std::pair<int, int> edge;
	AStar_easy(vector<easymath::XY> &locations, vector<edge> &edge_array, vector<double> &weights):
		locations(locations),edge_array(edge_array),weights(weights)
	{
		// create graph
		g = mygraph_t(locations.size());
		weightmap = get(edge_weight, g);
		for(std::size_t j = 0; j < edge_array.size(); ++j) {
			edge_descriptor e;
			bool inserted;
			boost::tuples::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
			weightmap[e] = weights[j];
		}
	}

		// create connections on map
	AStar_easy(list<AStar_easy::vertex> &high_path,vector<vector<bool> > *obstacle_map, vector<vector<int> > *membership_map){
		// Get 8-connected grid
		get8GridEdges(high_path,obstacle_map,membership_map); // populate reachable_locs, lookup, edge_array

		weights = vector<double>(edge_array.size(),1.0);
		g = mygraph_t(locations.size());
		weightmap = get(edge_weight, g);
		for(std::size_t j = 0; j < edge_array.size(); ++j) {
			edge_descriptor e;
			bool inserted;
			boost::tuples::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
			weightmap[e] = weights[j];
		}
	}
	~AStar_easy(void){};

	// translating to real things...
	map<XY,int> lookup;
	map<AStar_easy::vertex,XY> rlookup;



	// A* fundamentals
	vector<easymath::XY> locations; // physical locations of nodes
	vector<edge> edge_array; // connections in map
	vector<double> weights;
	mygraph_t g;

	// to delete
	WeightMap weightmap;

	list<vertex> search(vertex start, vertex goal){
		vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
		vector<cost> d(num_vertices(g));
		try {
			// call astar named parameter interface
			astar_search
				(g, start,
				distance_heuristic<mygraph_t, cost, vector<easymath::XY> >
				(locations, goal),
				predecessor_map(&p[0]).distance_map(&d[0]).
				visitor(astar_goal_visitor<vertex>(goal)));


		} catch(found_goal fg) { // found a path to the goal
			list<vertex> shortest_path;
			for(vertex v = goal;; v = p[v]) {
				shortest_path.push_front(v);
				if(p[v] == v)
					break;
			}
			return shortest_path;
		}
		return list<vertex>(1,start); // fail to find path: stay in one place
	}

	list<vertex> search(easymath::XY loc,easymath::XY goal){
		// Assumes mask already on high-level path: graph created externally: order enforced here
		// takes specific start and end points in space
		// lookup, looks up ID's for given nodes (loc,goal)
		if (lookup.count(loc) && lookup.count(goal))
			return search(vertex(lookup[loc]),vertex(lookup[goal]));
		else {
			printf("Goal not in mask. Aborting.");
			system("pause");
			exit(2);
		}
	}


	static vector<XY> get8GridNeighbors(int x, int y, vector<vector<bool> >* obstacle_map){
		int XDIM = obstacle_map->size();
		int YDIM = obstacle_map->at(0).size();
		vector<XY> neighbors(8);
		int n = 0;
		
		if (x>0 && y>0 && !obstacle_map->at(x-1)[y-1])				neighbors[n++] = XY(x-1,y-1);
		if (x>0 && !obstacle_map->at(x-1)[y])						neighbors[n++] = XY(x-1,y);
		if (x>0 && y<YDIM-1 && !obstacle_map->at(x-1)[y+1])			neighbors[n++] = XY(x-1,y+1);
		if (y>0 && !obstacle_map->at(x)[y-1])						neighbors[n++] = XY(x,y-1);
		if (y<YDIM-1 && !obstacle_map->at(x)[y+1])					neighbors[n++] = XY(x,y+1);
		if (x<XDIM-1 && y>0 && !obstacle_map->at(x+1)[y-1])			neighbors[n++] = XY(x+1,y-1);
		if (x<XDIM-1 && !obstacle_map->at(x+1)[y])					neighbors[n++] = XY(x+1,y);
		if (x<XDIM-1 && y<YDIM-1 && !obstacle_map->at(x+1)[y+1])	neighbors[n++] = XY(x+1,y+1);

		neighbors.resize(n);

	/*
		if (x>0 && y>0 && !obstacle_map->at(x-1)[y-1]) neighbors.push_back(XY(x-1,y-1));
		if (x>0 && !obstacle_map->at(x-1)[y]) neighbors.push_back(XY(x-1,y));
		if (x>0 && y<YDIM-1 && !obstacle_map->at(x-1)[y+1]) neighbors.push_back(XY(x-1,y+1));
		if (y>0 && !obstacle_map->at(x)[y-1]) neighbors.push_back(XY(x,y-1));
		if (y<YDIM-1 && !obstacle_map->at(x)[y+1]) neighbors.push_back(XY(x,y+1));
		if (x<XDIM-1 && y>0 && !obstacle_map->at(x+1)[y-1]) neighbors.push_back(XY(x+1,y-1));
		if (x<XDIM-1 && !obstacle_map->at(x+1)[y]) neighbors.push_back(XY(x+1,y));
		if (x<XDIM-1 && y<YDIM-1 && !obstacle_map->at(x+1)[y+1]) neighbors.push_back(XY(x+1,y+1));
		*/
		return neighbors;
	}

	void get8GridEdges(list<AStar_easy::vertex> &high_path, vector<vector<bool> > *obstacle_map, vector<vector<int> > *membership_map){
		
		set<XY> nodes; // set of nodes already found
		// create connections on map
		map<int,set<int> > member_connections;
		for (list<AStar_easy::vertex>::iterator i=high_path.begin(); i!=high_path.end(); i++){
			member_connections[*i] = set<int>();
			member_connections[*i].insert(*i); // insert self connection;
			if (std::next(i)!=high_path.end()){ // insert connection at next node
				member_connections[*i].insert(*std::next(i));
			}
		}
		
		int XDIM = obstacle_map->size();
		int YDIM = obstacle_map->at(0).size();
		for (int x=0; x<XDIM; x++){
			for (int y=0; y<YDIM; y++){
//				if (x==166 && y==187){
	//				printf("here!");
		//		}
				vector<XY> neighbors = get8GridNeighbors(x,y, obstacle_map);

				// TODO: CREATE GRAPH FROM REACHABLE NEIGHBORS
				int mem1 = membership_map->at(x)[y];

				if(mem1==int(high_path.front()) || member_connections.count(mem1)){
					for (int i=0; i<neighbors.size(); i++){
						int mem2 = membership_map->at(neighbors[i].x)[neighbors[i].y];
						if (mem1==mem2 || member_connections[mem1].count(mem2)){
							if (nodes.find(XY(x,y))==nodes.end()){
								lookup[XY(x,y)]=nodes.size(); // add to existing set
								rlookup[nodes.size()]=XY(x,y);
								nodes.insert(XY(x,y));
							}
							// add node AND EDGE
							if (nodes.find(neighbors[i])==nodes.end()){
								lookup[neighbors[i]]=nodes.size(); // add to existing set
								rlookup[nodes.size()]=neighbors[i];
								nodes.insert(neighbors[i]);
							}
							edge_array.push_back(AStar_easy::edge(lookup[XY(x,y)],lookup[neighbors[i]]));
						}
					}
				}
			}
		}

		locations = vector<XY>(nodes.size());
		int count=0;
		for (set<XY>::iterator it=nodes.begin(); it!=nodes.end(); it++){
			locations[count] = *it;
			count++;
		}
	}
};

