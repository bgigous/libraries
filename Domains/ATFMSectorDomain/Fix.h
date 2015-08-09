#pragma once

#include <list>
#include <vector>
#include "UAV.h"
#include "../../Planning/AStarManager.h"

class Fix{
public:
	Fix(XY loc, int ID, bool deterministic, AStarManager* planners);
	~Fix(){};
	

	std::list<UAV> generateTraffic(std::vector<Fix>* fixes,std::vector<std::vector<XY> > *pathTraces);
	void absorbTraffic(std::list<UAV>* UAVs);
	bool atDestinationFix(const UAV &u);
	int ID;
	AStarManager* planners; // for passing in for UAV creation
	bool is_deterministic;
	XY loc;
	const double p_gen;
	const double dist_thresh;
	static const int gen_frequency=10;
};