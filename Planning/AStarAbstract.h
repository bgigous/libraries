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

typedef double cost;

// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(Vertex goal, double xdim, double ydim): 
		m_goal(goal),
		XDIM(xdim),
		YDIM(ydim)
	{};
	CostType operator()(Vertex u)
	{
		int x1, y1, x2, y2;
		AStarAbstract::ind2sub(YDIM, u, x1, y1);
		AStarAbstract::ind2sub(YDIM, m_goal, x2, y2);
		
		double dx = x2-x1;
		double dy = y2-y1;
		return CostType(::sqrt(dx * dx + dy * dy)); // euclidean cost heuristic
	}
private:
//	LocMap m_location;
	Vertex m_goal;
	double XDIM, YDIM;
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


/*class zero_heuristic {
public:
	zero_heuristic(){};
		typedef adjacency_list<listS, vecS, undirectedS, no_property,
		property<edge_weight_t, cost> > mygraph_t;
  double operator()( mygraph_t::vertex_descriptor v) {
	  return 0.0;
  }
};*/

class AStarAbstract
{
public:
	typedef adjacency_list
		<listS, // edge container
		vecS,	// vertex container
		undirectedS,	// graph type is undirected -- SHOULD CHANGE THIS?
		no_property,
		property<edge_weight_t, cost> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef mygraph_t::vertex_iterator vertex_iterator;
	typedef std::pair<int, int> edge;
	AStarAbstract(vector<easymath::XY> &locations, vector<edge> &edge_array):
		locations(locations),edge_array(edge_array)
	{
		weights = matrix1d(edge_array.size(),1.0); // initialize all weights to 1
		// create graph
		g = mygraph_t(locations.size());
		weightmap = get(edge_weight, g);
		for(std::size_t j = 0; j < edge_array.size(); ++j) {
			edge_descriptor e;
			bool inserted;
			boost::tuples::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
			weightmap[e] = float(weights[j]);
		}
	}

	// create connections on map
	/*AStarAbstract(vector<vector<bool> > *obstacle_map, vector<vector<int> > *membership_map){
		// Get 8-connected grid
		get8GridEdges(obstacle_map,membership_map); // populate reachable_locs, lookup, edge_array

		weights = vector<double>(edge_array.size(),1.0);
		g = mygraph_t(locations.size());
		weightmap = get(edge_weight, g);
		for(std::size_t j = 0; j < edge_array.size(); ++j) {
			edge_descriptor e;
			bool inserted;
			boost::tuples::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
			weightmap[e] = float(weights[j]);
		}
	}*/
	~AStarAbstract(void){};
	
	// A* fundamentals
	vector<easymath::XY> locations; // physical locations of nodes
	vector<edge> edge_array; // connections in map
	vector<cost> weights;
	mygraph_t g;

	// to delete
	WeightMap weightmap;

	list<vertex> search(vertex start, vertex goal){
		vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
		vector<cost> d(num_vertices(g));
		try {
			astar_search
				(g, start,
				distance_heuristic<mygraph_t,cost>(goal,XDIM,YDIM),
				predecessor_map(&p[0]).distance_map(&d[0]).
				visitor(astar_goal_visitor<vertex>(goal)));

		} catch(found_goal fg) { // found a path to the goal
			(void)fg;
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
		// Takes specific start and end points in space
		
		int v1 = sub2ind((int)loc.x,(int)loc.y,XDIM,YDIM);
		int v2 = sub2ind((int)goal.x,(int)goal.y,XDIM,YDIM);

		if (nodes.count(loc) && nodes.count(goal))
			return search(vertex(v1),vertex(v2));
		else {
			printf("Goal not in mask. Aborting.");
			system("pause");
			exit(2);
		}
	}

	/*
	vector<XY> search(list<AStarAbstract::vertex> high_path, easymath::XY loc,easymath::XY goal){
		// Assumes mask already on high-level path: graph created externally: order enforced here
		// takes specific start and end points in space
		
		add_boundaries(high_path); // ADD IMPACT OF HIGH-LEVEL PATH
		
		int v1 = sub2ind(int(loc.x),int(loc.y),XDIM,YDIM);
		int v2 = sub2ind(int(goal.x),int(goal.y),XDIM,YDIM);
		list<vertex> path_output_vertices;

		if (nodes.count(loc) && nodes.count(goal)){
			path_output_vertices = search(vertex(v1),vertex(v2));
			remove_boundaries(high_path); // REMOVE IMPACT OF HIGH-LEVEL PATH
			vector<XY> path_output(path_output_vertices.size());
			int i=0;
			for (vertex  v : path_output_vertices){
				int x,y;
				ind2sub(YDIM,v,x,y);
				path_output[i++] = XY(x,y);
			}
			return path_output;
		}
		else {
			printf("Goal not in mask. Aborting.");
			system("pause");
			exit(2);
		}
	}*/


	/*
	static 	vector<XY> get8GridNeighbors(int x, int y, vector<vector<bool> >* obstacle_map){
		int XDIM = obstacle_map->size();
		int YDIM = obstacle_map->at(0).size();
		vector<XY> neighbors(8);
		int n = 0;

		if (x>0	&& y>0 && !obstacle_map->at(x-1)[y-1])				neighbors[n++] = XY(x-1,y-1);
		if (x>0	&& !obstacle_map->at(x-1)[y])						neighbors[n++] = XY(x-1,y);
		if (x>0	&& y<YDIM-1	&&	!obstacle_map->at(x-1)[y+1])		neighbors[n++] = XY(x-1,y+1);
		if (y>0	&& !obstacle_map->at(x)[y-1])						neighbors[n++] = XY(x,y-1);
		if (y<YDIM-1 &&	!obstacle_map->at(x)[y+1])					neighbors[n++] = XY(x,y+1);
		if (x<XDIM-1 &&	y>0	&&	!obstacle_map->at(x+1)[y-1])		neighbors[n++] = XY(x+1,y-1);
		if (x<XDIM-1 &&	!obstacle_map->at(x+1)[y])					neighbors[n++] = XY(x+1,y);
		if (x<XDIM-1 &&	y<YDIM-1 &&	!obstacle_map->at(x+1)[y+1])	neighbors[n++] = XY(x+1,y+1);

		neighbors.resize(n);

		return neighbors;
	}*/

	static int sub2ind(int r, int c, int m, int n){ 
		// 1-index!
		r++;
		c++;
		return (c-1)*m+r;
	}
	static void ind2sub(int cols, int ind, int &r, int &c){
		//0-indexed: ind-1 always called
		r = (ind-1)%cols;
		c = int(floor((ind-1)/cols));
	}
	int XDIM, YDIM; // dimensions of the map
	set<XY> nodes; // set of nodes already found

	// better to add back in the boundaries?

	/*
	map<pair<int,int>,vector<edge> > boundary_edges; // lists boundary connections between different sectors

	void add_boundaries(list<AStarAbstract::vertex> &high_path){
		
		//TODO: do other things here that influence graph creation
		for (list<AStarAbstract::vertex>::iterator i=high_path.begin(); i!=prev(high_path.end()); i++){
			vector<edge> b = boundary_edges[pair<int,int>(*i,*std::next(i))];
			for (edge e : b){
				edge_array.push_back(e);
				
				// add it into the graph!
				edge_descriptor desc;
				bool inserted;
				boost::tuples::tie(desc, inserted) = add_edge(e.first, e.second, g);
				weightmap[desc] = 1.0; // HARDCODING
			}
		}

	}
	

	void remove_boundaries(list<AStarAbstract::vertex> &high_path){
		for (list<AStarAbstract::vertex>::iterator i=high_path.begin(); i!=prev(high_path.end()); i++){
			vector<edge> b = boundary_edges[pair<int,int>(*i,*std::next(i))];
			for (edge e: b){
				remove_edge(e.first,e.second,g);
			}
		}
	}

	void get8GridEdges(vector<vector<bool> > *obstacle_map, vector<vector<int> > *membership_map){
		// Gets the 8-grid edges broken up by sectors specified by "membership map".
		// Reconnection of the sectors is based on the high-level plan during the search, and connects
		// relevant edges in boundary_edges container.

		XDIM = obstacle_map->size();
		YDIM = obstacle_map->at(0).size();

		// create connections on map
		for (int x=0; x<XDIM; x++){
			for (int y=0; y<YDIM; y++){
				if (!obstacle_map->at(x)[y]){
					nodes.insert(XY(x,y)); // add to list of possible nodes, if not in an obstacle
					vector<XY> neighbors = get8GridNeighbors(x,y, obstacle_map);
					
					for (XY n: neighbors){
						edge e;
						e.first = sub2ind(x,y,YDIM,XDIM);
						e.second = sub2ind((int)n.x,(int)n.y,YDIM,XDIM);

						int m1 = (int)membership_map->at((unsigned int)x)[(unsigned int)y];
						int m2 = (int)membership_map->at((unsigned int)n.x)[(unsigned int)n.y];

						if (m1==m2){
							edge_array.push_back(e);
						} else {
							boundary_edges[edge(m1,m2)].push_back(e);
						}
					}

				}
			}
		}

		locations = vector<XY>(nodes.size());
		int count=0;
		for (XY n : nodes){
			locations[count++] = n;
		}
	}
	*/
};

