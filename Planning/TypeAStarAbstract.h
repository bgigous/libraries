#pragma once
#include "AStarAbstract.h"
#include "AStarGrid.h"
#include "../Math/Matrix.h"
#include "../FileIO/easyio/easyio.h"
#include <memory>


/**
* Manages AStarAbstract usage for different types in the system
*/

using namespace Numeric_lib;
using namespace std;

class TypeAStarAbstract
{
public:
	typedef pair<int,int> Edge;
	typedef Matrix<bool,2> barrier_grid;

	TypeAStarAbstract(void);
	TypeAStarAbstract(int n_types, std::vector<Edge> edges, vector<XY> verticesFile);
	TypeAStarAbstract(string edgesFile, string verticesFile, int n_types);
	TypeAStarAbstract(int n_vertices, int n_types, double gridSizeX, double gridSizeY);
	~TypeAStarAbstract(void);

	// A* modification functions
	void setCostMaps(matrix2d agent_actions);
	list<int> search(int mem1, int mem2, int type_ID);
	
	// Accessor functions
	int getMembership(easymath::XY pt);
	XY getLocation(int sectorID);
	vector<Edge>& getEdges();
	int getNAgents();

private: 
	vector<Edge> edges;
	int n_types;
	map<XY, int> loc2mem; // maps location to membership
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
	std::vector<AStarAbstract*> Astar_highlevel;
	
	// Helpers/translators
	bool intersectsExistingEdge(pair<int, int> candidate,vector<XY> agentLocs);
	bool fullyConnected(vector<XY> agentLocs);
	void initializeTypeLookupAndDirections(vector<XY> agentLocs);
	matrix2d sectorTypeVertex2SectorTypeDirection(matrix2d agent_actions);

};

