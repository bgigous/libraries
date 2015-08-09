#include "AStarManager.h"


AStarManager::AStarManager(void)
{
}


AStarManager::~AStarManager(void)
{
	
	delete obstacle_map;
	delete membership_map;
	// Delete all pointers: recreate later...
	for (grid_lookup::iterator it=m2astar.begin(); it!=m2astar.end(); it++){
		for (std::map<int,AStar_grid*>::iterator inner=it->second.begin(); inner!=it->second.end(); inner++){
			delete inner->second;
		}
	}
	clear_all(Astar_highlevel);
}
