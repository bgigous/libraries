#include "Sector.h"


Sector::Sector(XY xy, int sectorIDSet, int n_agents): xy(xy), sectorID(sectorIDSet), n_agents(n_agents)
{
	// ADD IN NUMBER OF AGENTS SOMEWHERE...;
	total_loads = matrix2d(n_agents,matrix1d(UAV::NTYPES,0.0));
}