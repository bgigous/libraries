#pragma once
#include "../../libraries/Planning/AStar_easy.h"


/**
* A class to manage multiple A* instances that balances space and storage
*/


class AStarManager
{
public:
	
	AStarManager(void);
	~AStarManager(void);
	/*vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	// vector<double> weights; // old
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;*/
};

