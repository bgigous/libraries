#include "Sector.h"

Sector::Sector(easymath::XY xy, int sectorIDSet, int n_agents, std::vector<int> connections, UTMModes* params): 
	xy(xy), 
	ID(sectorIDSet), 
	connections(connections)
{
}