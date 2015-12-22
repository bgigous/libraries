#pragma once

#include "UAV.h"

class Sector{
public:
	// An area of space that contains some fixes
	Sector(XY xy, int sectorIDset, int n_agents, std::vector<int> connections, UTMModes* params);
	Sector(){}; // default constructor
	~Sector(){};

	matrix1d conflicts;
	int steps;
	vector<int> connections;
	XY xy; // sector center

private:
	int sectorID; // the identifier for this sector
};
