#pragma once
#include "LinkGraph.h"
#include "../FileIO/FileIn.h"
#include "../Math/easymath.h"
#include "../Planning/RAGS.h"


/**
* Manages LinkGraph usage for different types in the system
*/


class TypeGraphManager
{
public:
	typedef std::pair<int,int> edge;
	typedef std::vector<std::vector<bool> > barrier_grid;

	TypeGraphManager(void);
	TypeGraphManager(int n_types, std::vector<edge> edges, std::vector<easymath::XY> verticesFile);
	TypeGraphManager(std::string edgesFile, std::string verticesFile, int n_types);
	TypeGraphManager(int n_vertices, int n_types, double gridSizeX, double gridSizeY);
	~TypeGraphManager(void);

	// A* modification functions
	void setCostMaps(matrix2d agent_actions);
	std::list<int> astar(int mem1, int mem2, int type_ID);
	std::list<int> RAGS(int mem1, int mem2, int type_ID);

	// Accessor functions
	int getMembership(easymath::XY pt);
	easymath::XY getLocation(int sectorID);
	std::vector<edge> getEdges();
	int getNVertices();

	void print_graph(std::string file_path){
		Graph_highlevel[0]->print_graph_to_file(file_path);
	}

private:
	std::vector<edge> edges;
	int n_types;
	std::map<easymath::XY, int> loc2mem; // maps location to membership
	std::vector<LinkGraph*> Graph_highlevel;

	// Helpers/translators
	bool intersectsExistingEdge(edge candidate,std::vector<easymath::XY> agentLocs);
	bool fullyConnected(std::vector<easymath::XY> agentLocs);
	void initializeTypeLookupAndDirections(std::vector<easymath::XY> agentLocs);

	struct rags_info{
        std::vector<std::vector<double>> probDist;
        std::vector<std::vector<double>> points;
        void calc_prob_dist(double cost, int type);
        std::vector<double> get_prob_dist(int type);
	};
	std::vector<rags_info> rags_graph_data;
};

