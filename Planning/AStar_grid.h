#pragma once
/**
 * Example use of boost::astar_search_no_init on an infinite, implicitly-defined graph.
 *
 * The graph type used here is GridXYGraph, representing an infinite grid of squares.  Each
 * square is connected to its eight neighbors; however, the example shows how to use
 * boost::filtered_graph to make the search take place only along orthogonal edges.
 */

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/operators.hpp>
#include <boost/ref.hpp>


class AStar_grid{
public:
namespace Direction
{
	enum id
	{
		MIN = 0,
		N = MIN, S, E, W, NW, NE, SE, SW, NONE
	};
}

struct GridXY : public boost::additive<GridXY,
	boost::totally_ordered<GridXY,
	boost::equivalent<GridXY>
> >
{
	typedef int X;
	typedef int Y;
	X x;
	Y y;

	GridXY neighbor(Direction::id direction) const
	{
		using namespace Direction;

		int dx = 0, dy = 0;
		switch (direction)
		{
		case NW:
		case W:
		case SW:
			dx = -1;
			break;
		case NE:
		case E:
		case SE:
			dx = 1;
		}
		switch (direction)
		{
		case NW:
		case N:
		case NE:
			dy = -1;   
			break;
		case SW:
		case S:
		case SE:
			dy = 1;
		}
		GridXY const neighbor(x + dx, y + dy);
		return neighbor;
	}

	std::set<GridXY> allNeighbors() const
	{
		std::set<GridXY> neighbors;

		for (int dx = -1; dx <= 1; ++dx)
			for (int dy = -1; dy <= 1; ++dy)
				neighbors.insert(GridXY(x+dx,y+dy));

		return neighbors;
	}



	GridXY(X x, Y y)
		: x(x)
		, y(y)
	{
	}

	bool adjacentTo(GridXY const& that) const
	{
		return abs(x - that.x) <= 1 && abs(y - that.y) <= 1;
	}

	GridXY & operator=(GridXY const& that)
	{
		x = that.x;
		y = that.y;
		return *this;
	}

	GridXY & operator+=(GridXY const& that)
	{
		x += that.x;
		y += that.y;
		return *this;
	}

	bool operator<(GridXY const& that) const
	{
		return x < that.x || (x == that.x && y < that.y);
	}

};

std::ostream & operator<<(std::ostream & os, GridXY const& xy);

struct neighbor_iterator;

/*
 * Model of:
 *  * Graph
 *  * IncidenceGraph
 */
struct GridXYGraph
{
	GridXYGraph(){};

    // Graph concept requirements
    typedef GridXY                                vertex_descriptor;
    typedef std::pair<GridXY, GridXY>                 edge_descriptor;
    typedef boost::undirected_tag             directed_category;
    typedef boost::disallow_parallel_edge_tag edge_parallel_category;
    typedef boost::incidence_graph_tag        traversal_category;

    // IncidenceGraph concept requirements
    typedef neighbor_iterator          out_edge_iterator;
    typedef int                        degree_size_type;
};

namespace boost
{
    template <> struct graph_traits<GridXYGraph>
    {
        typedef GridXYGraph G;

        typedef G::vertex_descriptor      vertex_descriptor;
        typedef G::edge_descriptor        edge_descriptor;
        typedef G::out_edge_iterator      out_edge_iterator;

        typedef G::directed_category      directed_category;
        typedef G::edge_parallel_category edge_parallel_category;
        typedef G::traversal_category     traversal_category;

        typedef G::degree_size_type       degree_size_type;

        typedef void in_edge_iterator;
        typedef void vertex_iterator;
        typedef void vertices_size_type;
        typedef void edge_iterator;
        typedef void edges_size_type;
    };
}

// IncidenceGraph concept requirements
std::pair<GridXYGraph::out_edge_iterator, 
GridXYGraph::out_edge_iterator> out_edges(GridXYGraph::vertex_descriptor v, GridXYGraph const& g);
GridXYGraph::degree_size_type out_degree(GridXYGraph::vertex_descriptor v, GridXYGraph const& g);
GridXYGraph::vertex_descriptor source(GridXYGraph::edge_descriptor e, GridXYGraph const& g);
GridXYGraph::vertex_descriptor target(GridXYGraph::edge_descriptor e, GridXYGraph const& g);

// Iterator
struct neighbor_iterator : 
    public boost::iterator_facade<neighbor_iterator,
                                  std::pair<GridXY,GridXY>,
                                  boost::forward_traversal_tag,
                                  std::pair<GridXY,GridXY> >
{
public:
	neighbor_iterator(){};

    bool equal(neighbor_iterator const& that) const { return operator==(that); }
    void increment() { operator++(); }

	

neighbor_iterator(GridXY xy, Direction::id direction)
: xy(xy)
, direction(direction)
{
}

neighbor_iterator & operator=(neighbor_iterator const& that)
{
    xy = that.xy;
    direction = that.direction;
    return *this;
}

std::pair<GridXY,GridXY> operator*() const
{
    std::pair<GridXY,GridXY> const retval = std::make_pair(xy, xy.neighbor(direction));
    return retval;
}

void operator++()
{
    direction = static_cast<Direction::id>(int(direction) + 1);
}

bool operator==(neighbor_iterator const& that) const
{
    return xy == that.xy && direction == that.direction;
}

private:
    GridXY xy;
    Direction::id direction;
};


struct orthogonal_only;
template <typename Graph> class distance_heuristic;

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(GridXY goal) : m_goal(goal) {};

    virtual void examine_vertex(GridXY xy, GridXYGraph const& g) {
        std::cout << "Exploring " << xy << "..." << std::endl;
        if(xy == m_goal)
            throw found_goal();
    }
    virtual void examine_vertex(GridXY xy, boost::filtered_graph<GridXYGraph, orthogonal_only> const& g) {
        std::cout << "Exploring " << xy << "..." << std::endl;
        if(xy == m_goal)
            throw found_goal();
    }
private:
    GridXY m_goal;
};

template <typename K, typename V>
class default_map
{
public:
    typedef K key_type;
    typedef V data_type;
    typedef std::pair<K,V> value_type;

    default_map(V const& defaultValue)
        : defaultValue(defaultValue)
    {}

    V & operator[](K const& k)
    {
        if (m.find(k) == m.end())
        {
            m[k] = defaultValue;
        }
        return m[k];
    }

private:
    std::map<K,V> m;
    V const defaultValue;
};

struct PredecessorMap
{
    PredecessorMap() {}
    PredecessorMap(PredecessorMap const& that) : m(that.m) {}

    typedef GridXY key_type;
    typedef GridXY value_type;
    typedef GridXY & reference_type;
    typedef boost::read_write_property_map_tag category;

    GridXY & operator[](GridXY xy) { return m[xy]; }

    std::map<GridXY,GridXY> m;
};

GridXY get(PredecessorMap const& pm, GridXY xy)
{
    std::map<GridXY,GridXY>::const_iterator found = pm.m.find(xy);
    return (found != pm.m.end()) ? found->second : xy;
}

void put(PredecessorMap & pm, GridXY key, GridXY value)
{
    pm.m[key] = value;
}

// Filter used to traverse grid only along orthogonal (non-diagonal) edges.
struct orthogonal_only
{
    typedef std::pair<GridXY,GridXY> Edge;
    bool operator()(Edge const& edge) const
    {
        return edge.first.x == edge.second.x || edge.first.y == edge.second.y;
    }
};

// Euclidean distance heuristic (square root omitted)
template <typename Graph>
class distance_heuristic : public boost::astar_heuristic<Graph, int>
{
public:
    distance_heuristic(GridXY goal)
        : m_goal(goal) {}
    unsigned operator()(GridXY xy)
    {
        int dx = m_goal.x - xy.x;
        int dy = m_goal.y - xy.y;
        unsigned retval = static_cast<unsigned>(dx * dx + dy * dy);
        return retval;
    }
private:
    GridXY m_goal;
};


std::pair<GridXYGraph::out_edge_iterator, GridXYGraph::out_edge_iterator> 
out_edges(GridXYGraph::vertex_descriptor v,
          GridXYGraph const& g)
{
    return std::make_pair(
        GridXYGraph::out_edge_iterator(v, Direction::MIN), 
        GridXYGraph::out_edge_iterator(v, Direction::NONE) );
}

GridXYGraph::degree_size_type 
out_degree(GridXYGraph::vertex_descriptor v,
           GridXYGraph const& g)
{
    return v.allNeighbors().size();
}

GridXYGraph::vertex_descriptor 
source(GridXYGraph::edge_descriptor e,
       GridXYGraph const& g)
{
    return e.first;
}

GridXYGraph::vertex_descriptor target(
    GridXYGraph::edge_descriptor e,
    GridXYGraph const& g)
{
    return e.second;
}


std::ostream & operator<<(std::ostream & os, GridXY const& xy)
{
    os << "(" << xy.x << "," << xy.y << ")";
    return os;
}


/*
int main(int argc, char **argv)
{
    GridXYGraph baseGraph;
    boost::filtered_graph<GridXYGraph, orthogonal_only> g(baseGraph, orthogonal_only());
    //BOOST_CONCEPT_ASSERT((IncidenceGraphConcept< boost::filtered_graph<GridXYGraph, orthogonal_only> >));

    GridXY start(0,0);
    GridXY goal(5,7);

    std::cout << "Start vertex: " << start << std::endl;
    std::cout << "Goal vertex: " << goal << std::endl;

    PredecessorMap p;
    typedef boost::associative_property_map< default_map<GridXY,unsigned> > DistanceMap;
    typedef default_map<GridXY,unsigned> WrappedDistanceMap;
    WrappedDistanceMap wrappedMap =         WrappedDistanceMap(std::numeric_limits<unsigned>::max());
    wrappedMap[start] = 0;
    DistanceMap d = DistanceMap(wrappedMap);

    try {
        astar_search_no_init(g, 
            start,
            distance_heuristic<GridXYGraph>(goal)
            , visitor(astar_goal_visitor(goal))
            . distance_map(d)
            . predecessor_map(boost::ref(p))
            . weight_map(boost::associative_property_map< default_map<std::pair<GridXY,GridXY>,unsigned> >(
                default_map<std::pair<GridXY,GridXY>,unsigned>(1)))
            . vertex_index_map(boost::associative_property_map< std::map<GridXY,unsigned> >(std::map<GridXY,unsigned>()))
            . rank_map(boost::associative_property_map< std::map<GridXY,unsigned> >(std::map<GridXY,unsigned>()))
            . color_map(boost::associative_property_map< std::map<GridXY,boost::default_color_type> >(
                std::map<GridXY,boost::default_color_type>()))
            . distance_compare(std::less<unsigned>())
            . distance_combine(std::plus<unsigned>())
            );
    } catch(found_goal const&) { // found a path to the goal
        std::list<GridXY> shortest_path;
        for(GridXY xy = goal;; xy = p[xy]) {
            shortest_path.push_front(xy);
            if(p[xy] == xy)
                break;
        }
        std::cout << "Shortest path from " << start << " to "
            << goal << ": ";
        std::list<GridXY>::iterator spi = shortest_path.begin();
        std::cout << start;
        for(++spi; spi != shortest_path.end(); ++spi) 
            std::cout << " -> " << (*spi);
        std::cout << std::endl;
        return 0;
    }

    std::cout << "Didn't find a path from " << start << "to"
        << goal << "!" << std::endl;
    return 0;
}*/




/*
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <iostream>
#include <limits>
#include "../Math/easymath.h"
#include <sstream>
#include <fstream>
#include "../Math/Matrix.h"

using namespace easymath;
using namespace Numeric_lib;
using namespace std;



class mt{ // "maze types"
public:
	// Distance traveled in the maze
	typedef double distance;

	static const int GRID_RANK=2;
	typedef boost::grid_graph<GRID_RANK> grid;
	typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
	typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

	// A hash function for vertices.
	struct vertex_hash:std::unary_function<vertex_descriptor, std::size_t> {
		std::size_t operator()(vertex_descriptor const& u) const {
			std::size_t seed = 0;
			boost::hash_combine(seed, u[0]);
			boost::hash_combine(seed, u[1]);
			return seed;
		}
	};

	typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
	typedef vector<vertex_descriptor> vertex_vector;
	typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
		filtered_grid;

};

// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:
	public boost::astar_heuristic<mt::filtered_grid, double>
{
public:
  //euclidean_heuristic(mt::vertex_descriptor goal):m_goal(goal) {};
	euclidean_heuristic(){};

  double operator()(mt::vertex_descriptor v) {
	  return sqrt(pow(double(m_goal[0]) - double(v[0]), 2) + pow(double(m_goal[1]) - double(v[1]), 2));
  }

  mt::vertex_descriptor m_goal;
};

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:

		// Exception thrown when the goal vertex is found
	struct found_goal {};

	// Visitor that terminates when we find the goal vertex
	struct astar_goal_visitor:public boost::default_astar_visitor {
	//	astar_goal_visitor(mt::vertex_descriptor goal):m_goal(goal) {};
		astar_goal_visitor(){};

		void examine_vertex(mt::vertex_descriptor u, const mt::filtered_grid&) {
			if (u == m_goal)
				throw found_goal();
		}

		mt::vertex_descriptor m_goal;
	};


	friend std::ostream& operator<<(std::ostream&, const maze&);
	friend maze random_maze(std::size_t, std::size_t);

	// CREATION FUNCTIONS
	maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {};
	maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),
		m_barrier_grid(create_barrier_grid()) {};

	maze(Matrix<bool,2> *obstacle_map):
		m_grid(create_grid(obstacle_map->dim1(),obstacle_map->dim2())),
		m_barrier_grid(create_barrier_grid())
	{
		int v_index = 0;
		// NOTE: V_INDEX COUNT ASSUMES YOU'RE GOING THROUGH THE OBSTACLES DIFFERENTLY...
		//for (std::vector<std::vector<bool> >::iterator obs = obstacle_map->begin(); obs!=obstacle_map->end(); obs++){
			//for (std::vector<bool>::iterator o = obs->begin(); o!=obs->end(); o++){
		for (int y=0; y<obstacle_map->dim2(); y++){
			for (int x=0; x<obstacle_map->dim1(); x++){
				if ((*obstacle_map)(x,y)){ // there is an obstacle!
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(std::vector<std::vector<bool> >* obstacle_map):
		m_grid(create_grid(obstacle_map->size(),obstacle_map->begin()->size())),
		m_barrier_grid(create_barrier_grid())
	{
		// SOURCE AND GOAL DECIDED IN SEARCH!

		int v_index = 0;
		// NOTE: V_INDEX COUNT ASSUMES YOU'RE GOING THROUGH THE OBSTACLES DIFFERENTLY...
		//for (std::vector<std::vector<bool> >::iterator obs = obstacle_map->begin(); obs!=obstacle_map->end(); obs++){
			//for (std::vector<bool>::iterator o = obs->begin(); o!=obs->end(); o++){
		for (unsigned int y=0; y<obstacle_map->begin()->size(); y++){
			for (unsigned int x=0; x<obstacle_map->size(); x++){
				if (obstacle_map->at(x)[y]){ // there is an obstacle!
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(std::vector<std::vector<bool> >* obstacle_map, std::vector<std::vector<int> >* membership_map, int m1, int m2):
		m_grid(create_grid(obstacle_map->size(),obstacle_map->begin()->size())),
		m_barrier_grid(create_barrier_grid())
	{
		// This map will only show areas of membership. Others are seen as barriers.

		int v_index = 0;
		for (unsigned int x=0; x<obstacle_map->begin()->size(); x++){
			for (unsigned int y=0; y<obstacle_map->size(); y++){
				if (obstacle_map->at(x)[y] || (membership_map->at(x)[y]!=m1 && membership_map->at(x)[y]!=m2)){ // there is an obstacle, or wrong membership
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	maze(Matrix<bool,2> *obstacle_map, Matrix<int,2> *members, int m1, int m2):
		m_grid(create_grid(obstacle_map->dim1(), obstacle_map->dim2())),
		m_barrier_grid(create_barrier_grid())
	{
		// This map will only show areas of membership. Others are seen as barriers.
		int v_index = 0;
		for (int y=0; y<obstacle_map->dim2(); y++){
			for (int x=0; x<obstacle_map->dim1(); x++){
				if (((*obstacle_map)(x,y)) || 
					((*members)(x,y)!=m1 && (*members)(x,y)!=m2)){ // there is an obstacle, or wrong membership
					mt::vertex_descriptor u = vertex(v_index,m_grid);
					m_barriers.insert(u); // insert a barrier!
				}
				v_index++; // increment v_index even if no barrier added!
			}
		}
	}

	// The length of the maze along the specified dimension.
	mt::vertices_size_type length(std::size_t d) const {return m_grid.length(d);}

	bool has_barrier(mt::vertex_descriptor u) const {
		return m_barriers.find(u) != m_barriers.end();
	}

	double solve(int xsource, int ysource, int xgoal, int ygoal){
		boost::static_property_map<mt::distance> weight(1);
		// The predecessor map is a vertex-to-vertex mapping.
		typedef boost::unordered_map<mt::vertex_descriptor,
			mt::vertex_descriptor,
			mt::vertex_hash> pred_map;
		pred_map predecessor;
		boost::associative_property_map<pred_map> pred_pmap(predecessor);
		// The distance map is a vertex-to-distance mapping.
		typedef boost::unordered_map<mt::vertex_descriptor,
			mt::distance, mt::vertex_hash> dist_map;
		dist_map distance;
		boost::associative_property_map<dist_map> dist_pmap(distance);

		mt::vertex_descriptor s = {{xsource, ysource}};
		mt::vertex_descriptor g = {{xgoal, ygoal}};
		heuristic.m_goal = g;
		visitor.m_goal = g;
		m_solution.clear();

		try {
			astar_search(m_barrier_grid, s, heuristic,
				boost::weight_map(weight).
				predecessor_map(pred_pmap).
				distance_map(dist_pmap).
				visitor(visitor) );
		} catch(found_goal fg) {
			(void)fg;
			// Walk backwards from the goal through the predecessor chain adding
			// vertices to the solution path.
			for (mt::vertex_descriptor u = g; u != s; u = predecessor[u])
				m_solution.push_back(u);
			m_solution.push_back(s);
			m_solution_length = distance[g];
			//return true;
			return m_solution_length;
		}
		double maxdist = DBL_MAX;
		return maxdist;
	}

	std::vector<XY> get_solution_path(XY source, XY goal){
		solve((int)source.x,(int)source.y, (int)goal.x, (int)goal.y);

//		printf("size = %i\n",m_solution.size());

		std::vector<XY> soln;
		for (mt::vertex_vector::iterator it=m_solution.begin(); it!=m_solution.end(); it++){
			soln.push_back(XY(it->front(), it->back()));
		}
		return soln;
	}

	bool solved() const {return !m_solution.empty();}
	bool solution_contains(mt::vertex_descriptor u) const {
		return std::find(m_solution.begin(),m_solution.end(),u) != m_solution.end();
	}

	euclidean_heuristic heuristic;
	astar_goal_visitor visitor;

	
	// The length of the solution path
	mt::distance m_solution_length;

	void printMap(std::string dir, int label1, int label2){
		stringstream ss;
		ss << dir << label1 << "-" <<label2 << ".csv";
		ofstream output(ss.str());

		
		for (unsigned int i = 0; i<length(0); i++){
			for (unsigned int j = 0; j < length(1); j++) {
				// Put the character representing this point in the maze grid.
				mt::vertex_descriptor u = {{i, j}};
				if (solution_contains(u))
					output << 2 << ",";
				else if (has_barrier(u))
					output << 1 << ",";
				else
					output << 0 << ",";
			}
			output << std::endl;
		}
	}
		// The vertices on a solution path through the maze
	mt::vertex_vector m_solution;

private:
	// Create the underlying rank-2 grid with the specified dimensions.
	mt::grid create_grid(std::size_t x, std::size_t y) {
		boost::array<std::size_t, mt::GRID_RANK> lengths = { {x, y} };
		return mt::grid(lengths);
	}

	// Filter the barrier vertices out of the underlying grid.
	mt::filtered_grid create_barrier_grid() {
		return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
	}

	// The grid underlying the maze
	mt::grid m_grid;
	// The underlying maze grid with barrier vertices filtered out
	mt::filtered_grid m_barrier_grid;
	// The barriers in the maze
	mt::vertex_set m_barriers;

	
};

static std::ostream& operator<<(std::ostream& output, const maze& m) {
		// Header
		std::string BARRIER = "#";

		//for (mt::vertices_size_type i = 0; i < m.length(0)+2; i++)

		unsigned int START = 78;
		unsigned int END = START+78;

		for (mt::vertices_size_type i = START; i < END+2; i++)
			output << BARRIER;
		output << std::endl;
		// Body
		for (unsigned int y = 0; y<m.length(1); y++){//m.length(1)-1; y >= 0; y--) {
			// Enumerate rows in reverse order and columns in regular order so that
			// (0,0) appears in the lower left-hand corner.  This requires that y be
			// int and not the unsigned vertices_size_type because the loop exit
			// condition is y==-1.
			//for (mt::vertices_size_type x = 0; x < m.length(0); x++) { // hack to fit on screen
			for (mt::vertices_size_type x = START; x < END; x++) {
				// Put a barrier on the left-hand side.
				if (x == 0)
					output << BARRIER;
				// Put the character representing this point in the maze grid.
				mt::vertex_descriptor u = {{x, mt::vertices_size_type(y)}};
				if (m.solution_contains(u))
					output << ".";
				else if (m.has_barrier(u))
					output << BARRIER;
				else
					output << " ";
				// Put a barrier on the right-hand side.
				if (x == m.length(0)-1)
					output << BARRIER;
			}
			// Put a newline after every row except the last one.
			output << std::endl;
		}
		// Footer
		for (mt::vertices_size_type i = START; i < END+2; i++)
			output << BARRIER;
		if (m.solved())
			output << std::endl << "Solution length " << m.m_solution_length;
		return output;
}


class AStar_grid
{
public:
	AStar_grid(void);
	AStar_grid(std::vector<std::vector<bool> > *obstacle_map):
		m(maze(obstacle_map))
	{
	}
	AStar_grid(std::vector<std::vector<bool> > *obstacle_map, std::vector<std::vector<int> >* members, int m1, int m2):
		m(maze(obstacle_map,members,m1,m2))
	{
	}
	AStar_grid(Matrix<bool,2> *obstacle_map, Matrix<int,2> *members, int m1, int m2):
		m(maze(obstacle_map,members,m1,m2))
	{
	}
	AStar_grid(Matrix<bool,2> *obstacle_map):
		m(maze(obstacle_map))
	{
	}
	~AStar_grid(void){};
	maze m;
};*/

/*



// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
}


*/

