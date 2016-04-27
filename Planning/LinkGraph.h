// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_LINKGRAPH_H_
#define PLANNING_LINKGRAPH_H_

// Boost includes
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

// STL includes
#include <vector>
#include <list>
#include <utility>
#include <string>

// library includes
#include "../Math/easymath.h"
#include "../FileIO/FileOut.h"

typedef double cost;

// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
 public:
    typedef std::vector<easymath::XY> LocMap;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap locations, Vertex goal) :
        m_location(locations),
        m_goal(goal)
    {};
    CostType operator()(Vertex u) {
        return CostType(easymath::euclidean_distance(m_location[u],
            m_location[m_goal]));
    }
 private:
    LocMap m_location;
    Vertex m_goal;
    // double XDIM, YDIM;
};


struct found_goal {};  // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
 public:
    explicit astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& ) {
        if (u == m_goal)
            throw found_goal();
    }
 private:
    Vertex m_goal;
};

class LinkGraph {
 public:
    typedef boost::adjacency_list
        <boost::listS,      // edge container
        boost::vecS,        // vertex container
        boost::directedS,   // edge (u,v) can have a different weight than (v,u)
        boost::no_property,
        boost::property<boost::edge_weight_t, cost> > mygraph_t;
    // Note: m_edges is not populated
    // vertex is an int: corresponds to number in the locations list
    typedef mygraph_t::vertex_descriptor vertex;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;

    LinkGraph(std::vector<easymath::XY> locations_set,
        const std::vector<edge> &edge_array) :
        locations(locations_set) {
        // create graph
        g = mygraph_t(edge_array.begin(), edge_array.end(), locations.size());
        setWeights(matrix1d(edge_array.size(), 1.0));
    }

    // WEIGHT MODIFICATIONS
    matrix1d saved_weights;  // for blocking and unblocking sectors
    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
    void blockVertex(int vertexID) {
        // Makes it highly suboptimal to travel to a vertex
        saved_weights = getWeights();

        edge_iter ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            if ((*ei).m_target == vertex(vertexID))
                put(boost::edge_weight, g, *ei, 999999.99);
        }
    }

    void unblockVertex() {
        setWeights(saved_weights);
    }

    void setWeights(matrix1d weights) {
        // iterate over all edge descriptors...
        typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
        edge_iter ei, ei_end;
        int i = 0;

        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            put(boost::edge_weight, g, *ei, weights[i++]);
        }
    }

    matrix1d getWeights() {
        // iterate over all edge descriptors...
        typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
        edge_iter ei, ei_end;
        matrix1d weights;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            weights.push_back(get(boost::edge_weight, g, *ei));
        }
        return weights;
    }
    ~LinkGraph(void) {}

    // A* fundamentals
    mygraph_t g;
    std::vector<easymath::XY> locations;

    std::list<int> vertex2int(std::list<vertex> vertexpath) {
        std::list<int> intpath;
        while (vertexpath.size()) {
            intpath.push_back(vertexpath.front());
            vertexpath.pop_front();
        }
        return intpath;
    }

    std::list<int> astar(int start, int goal) {
        std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
        std::vector<cost> d(num_vertices(g));
        try {
            boost::astar_search
                (g, start,
                    distance_heuristic<mygraph_t, cost>(locations, goal),
                    boost::predecessor_map(&p[0]).distance_map(&d[0]).
                    visitor(astar_goal_visitor<vertex>(goal)));
        }
        catch (found_goal fg) {  // found a path to the goal
            (void)fg;
            std::list<vertex> shortest_path;
            for (vertex v = goal;; v = p[v]) {
                shortest_path.push_front(v);
                if (p[v] == v)
                    break;
            }
            return vertex2int(shortest_path);
        }
        // fail to find path: stay in one place
        return std::list<int>(1, static_cast<int>(start));
    }

    void print_graph_to_file(std::string file_path) {
        std::string CONNECTIONS_FILE = file_path + "connections.csv";
        std::string NODES_FILE = file_path + "nodes.csv";
        std::string EDGES_FILE = file_path + "edges.csv";

        std::vector<std::vector<bool> >
            connections_matrix(locations.size(),
                std::vector<bool>(locations.size(), false));

        edge_iter ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
            connections_matrix[(*ei).m_source][(*ei).m_target] = true;

        std::vector<edge> my_edges;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            my_edges.push_back(std::make_pair((*ei).m_source, (*ei).m_target));
        }

        FileOut::print_pair_container(locations, NODES_FILE);
        FileOut::print_pair_container(my_edges, EDGES_FILE);
        FileOut::print_vector(connections_matrix, CONNECTIONS_FILE);
    }
};
#endif  // PLANNING_LINKGRAPH_H_
