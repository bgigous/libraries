#include "Sector.h"

Sector::Sector(XY xy, int sectorIDSet, int n_agents, std::vector<int> connections, UTMModes* params): 
	xy(xy), 
	sectorID(sectorIDSet), 
	connections(connections)
{
	conflicts  = matrix1d(params->get_n_types(),0.0);
	steps = 0;
}