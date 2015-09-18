#pragma once

#include "UAV.h"
#include "../../Math/Matrix.h"
#include "../../Planning/AStar_grid.h"


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
class Sector{
public:
	// An area of space that contains some fixes
	int sectorID; // the identifier for this sector
	Sector(XY xy, int sectorIDset);
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center
	std::list<UAV*> toward; // the UAVs that are going toward the sector
	Demographics getLoad(){
		// Get demographic information about the UAVs traveling toward the sector
		Demographics load(UAV::NTYPES,0);
		for (UAV *u: toward){
			load[u->type_ID]++;
		}
		return load;
	}
};
