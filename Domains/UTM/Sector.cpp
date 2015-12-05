#include "Sector.h"


Sector::Sector(XY xy, int sectorIDSet, int n_agents, vector<int> connections): 
	xy(xy), 
	sectorID(sectorIDSet), 
	n_agents(n_agents), 
	connections(connections)
{
	conflicts  = matrix1d(UAV::NTYPES,0.0);
	steps = 0;
}