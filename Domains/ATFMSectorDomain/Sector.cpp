#include "Sector.h"


Sector::Sector(XY xy, int sectorIDSet): xy(xy), sectorID(sectorIDSet)
{
	total_loads = Demographics(UAV::NTYPES,0);
}