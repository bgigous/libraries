#include "AStarManager.h"


AStarManager::AStarManager(void)
{
}


AStarManager::~AStarManager(void)
{
	
	delete obstacle_map;
	delete membership_map;
	// Delete all pointers: recreate later...
	for (auto& it : m2astar){
		for (auto& inner : it.second){
			delete inner.second;
		}
	}
	//clear_all(Astar_highlevel);
}
