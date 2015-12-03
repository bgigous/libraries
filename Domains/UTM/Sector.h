#pragma once

#include "UAV.h"
#include "../../Math/Matrix.h"
#include "../../Planning/AStarGrid.h"
#include <memory>


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
class Sector{
public:
	// An area of space that contains some fixes
	int sectorID; // the identifier for this sector
	Sector(XY xy, int sectorIDset, int n_agents);
	int n_agents;
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center

	vector<int> conflicts;
	int steps;
};
