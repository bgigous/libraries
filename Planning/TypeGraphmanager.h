#pragma once
#include "LinkGraph.h"
//#include "GridGraph.h"
#include "../FileIO/FileIn.h"
#include <memory>
#include "../Math/easymath.h"


/**
* Manages LinkGraph usage for different types in the system
*/


class TypeGraphManager
{
public:
	typedef std::pair<int,int> Edge;
	typedef std::vector<std::vector<bool> > barrier_grid;

	TypeGraphManager(void);
	TypeGraphManager(int n_types, std::vector<Edge> edges, std::vector<easymath::XY> verticesFile);
	TypeGraphManager(std::string edgesFile, std::string verticesFile, int n_types);
	TypeGraphManager(int n_vertices, int n_types, double gridSizeX, double gridSizeY);
	~TypeGraphManager(void);

	// A* modification functions
	void setCostMaps(matrix2d agent_actions);
	std::list<int> astar(int mem1, int mem2, int type_ID);
	
	// Accessor functions
	int getMembership(easymath::XY pt);
	easymath::XY getLocation(int sectorID);
	std::vector<Edge> getEdges();
	int getNVertices();
	int getEdgeID(Edge e){
		return Graph_highlevel[0]->getEdgeID(e);
	}

private: 
	std::vector<Edge> edges;
	int n_types;
	std::map<easymath::XY, int> loc2mem; // maps location to membership
	std::vector<LinkGraph*> Graph_highlevel;
	
	// Helpers/translators
	bool intersectsExistingEdge(std::pair<int, int> candidate,std::vector<easymath::XY> agentLocs);
	bool fullyConnected(std::vector<easymath::XY> agentLocs);
	void initializeTypeLookupAndDirections(std::vector<easymath::XY> agentLocs);
};

