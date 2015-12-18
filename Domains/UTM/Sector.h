#pragma once

#include "UAV.h"
#include "../../Math/Matrix.h"
#include "../../Planning/GridGraph.h"
#include <memory>


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
class Sector{
public:
	// An area of space that contains some fixes
	int sectorID; // the identifier for this sector
	Sector(XY xy, int sectorIDset, int n_agents, vector<int> connections);
	int n_agents;
	vector<int> connections;
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center

	matrix1d conflicts;
	int steps;
};
