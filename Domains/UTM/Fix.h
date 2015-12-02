#pragma once

#include <list>
#include <vector>
#include <memory>
#include "UAV.h"
#include "../../Planning/AStarManager.h"

class Fix{
public:
	Fix(XY loc, int ID, AStarManager* planners);
	~Fix(){};
	
	
	enum TrafficMode{DETERMINISTIC, PROBABILISTIC};
	static const TrafficMode _traffic_mode=DETERMINISTIC;
	enum ArrivalMode{EXACT, THRESHOLD};
	const ArrivalMode _arrival_mode;

	std::list<std::shared_ptr<UAV> > generateTraffic(std::vector<Fix>* fixes);
	
	void absorbTraffic(std::list<UAV>* UAVs);
	bool atDestinationFix(const UAV &u);
	int ID;
	AStarManager* planners; // for passing in for UAV creation
	XY loc;

private:
	double p_gen;
	const double dist_thresh;
	
};