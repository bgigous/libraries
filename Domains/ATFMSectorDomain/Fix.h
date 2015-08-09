#pragma once

#include <list>
#include <vector>
#include "UAV.h"

class Fix{
public:
	Fix(XY loc, int ID, bool deterministic);
	~Fix(){};

	std::list<UAV> generateTraffic(std::vector<Fix>* fixes, barrier_grid* obstacle_map,std::vector<std::vector<XY> > *pathTraces);
	void absorbTraffic(std::list<UAV>* UAVs);
	bool atDestinationFix(const UAV &u);
	int ID;
	
	bool is_deterministic;
	XY loc;
	const double p_gen;
	const double dist_thresh;
	static const int gen_frequency=10;
};