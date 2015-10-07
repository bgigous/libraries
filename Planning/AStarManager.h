#pragma once
#include "AStar_easy.h"
#include "AStar_grid.h"
#include "../Math/Matrix.h"
#include <memory>


/**
* A class to manage multiple A* instances that balances space and storage
*/

using namespace Numeric_lib;
using namespace std;

class AStarManager
{
public:

	typedef std::map<int,std::map<int,AStar_grid*> > grid_lookup;
	typedef Matrix<bool,2> barrier_grid;
	typedef Matrix<int,2> ID_grid;
	typedef pair<int,int> Edge;

	AStarManager(void);
	~AStarManager(void);
	AStarManager(int n_types, vector<Edge> edges, Matrix<int,2>* membership_map, vector<XY> agent_locs);

	int getMembership(easymath::XY pt);
	void blockSector(int sectorID);
	void unblockSector();
	list<int> search(int type_ID, easymath::XY start_loc, easymath::XY end_loc);
	void reset();
	unsigned int n_types;
	void setCostMaps(vector<vector<double> > agent_actions);
	void resetGraphWeights(matrix2d weightset);
	
	matrix2d saved_weights; // for blocking and unblocking sectors
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;

	void printMasks();
};

