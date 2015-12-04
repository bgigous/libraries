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
	typedef vector<XY> LocMap;
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap locations, Vertex goal): 
		m_location(locations),
		m_goal(goal)
		//XDIM(xdim),
		//YDIM(ydim)
	{};
	CostType operator()(Vertex u)
	{
		int x1, y1, x2, y2;
		//AStarAbstract::ind2sub(YDIM, u, x1, y1);
		//AStarAbstract::ind2sub(YDIM, m_goal, x2, y2);
		x1 = m_location[u].x;
		y1 = m_location[u].x;
		x2 = m_location[m_goal].y;
		y2 = m_location[m_goal].y;


		double dx = x2-x1;
		double dy = y2-y1;
		return CostType(::sqrt(dx * dx + dy * dy)); // euclidean cost heuristic
	}
private:
	LocMap m_location;
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

class AStarAbstract
{
public:
	typedef adjacency_list
		<listS, // edge container
		vecS,	// vertex container
		directedS,	// graph type is directed: edge (u,v) can have a different weight than (v,u)
		no_property,
		property<edge_weight_t, cost> > mygraph_t;
	//typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex; // vertex is an int: corresponds to number in the locations list
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef mygraph_t::vertex_iterator vertex_iterator;
	typedef std::pair<int, int> edge;
	AStarAbstract(vector<XY> locations_set, vector<edge> &edge_array):
		locations(locations_set)
	{
		// create graph
		g = mygraph_t(locations.size());
		for(std::size_t j = 0; j < edge_array.size(); ++j) {
			edge_descriptor e;
			bool inserted;
			boost::tuples::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
		}
		setWeights(matrix1d(edge_array.size(),1.0));
	}

	void setWeights(matrix1d weights){
		// iterate over all edge descriptors...
		typedef graph_traits<mygraph_t>::edge_iterator edge_iter;
		std::pair<edge_iter, edge_iter> ep;
		edge_iter ei, ei_end;
		int i=0;
		for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei){
			put(edge_weight,g,*ei,weights[i++]);
		}
	}

	~AStarAbstract(void){};
	
	// A* fundamentals
	mygraph_t g;
	vector<XY> locations;

	list<vertex> search(vertex start, vertex goal){
		vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
		vector<cost> d(num_vertices(g));
		try {
			astar_search
				(g, start,
				distance_heuristic<mygraph_t,cost>(locations, goal),
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

	list<vertex> search(int v1, int v2){//easymath::XY loc,easymath::XY goal){
		return search(vertex(v1),vertex(v2));
	}
};

