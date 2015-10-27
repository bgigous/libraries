#include "Sector.h"


Sector::Sector(XY xy, int sectorIDSet): xy(xy), sectorID(sectorIDSet)
{
	// ADD IN NUMBER OF AGENTS SOMEWHERE...;
	total_loads = matrix2d(15,matrix1d(UAV::NTYPES,0.0));
}