#pragma once

#include "UTMModesAndFiles.h"
#include "../../Math/easymath.h"

class Sector{
public:
	// An area of space that contains some fixes
	Sector(easymath::XY xy, int sectorIDset, int n_agents, std::vector<int> connections, UTMModes* params);
	Sector(){}; // default constructor
	~Sector(){};

	matrix1d conflicts;
	int steps;
	std::vector<int> connections;
	easymath::XY xy; // sector center

private:
	int sectorID; // the identifier for this sector
};
