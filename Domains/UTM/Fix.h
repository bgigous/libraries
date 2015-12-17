#pragma once

#include <list>
#include <vector>
#include <memory>
#include "UAV.h"
#include "../../Planning/TypeAStarAbstract.h"
#include "../../Planning/SectorAStarGrid.h"

class Fix{
public:
	Fix(XY loc, int ID, TypeAStarAbstract* planners, SectorAStarGrid* lowPlanners, vector<Fix>* fixes);
	~Fix(){};
	
	
	enum TrafficMode{DETERMINISTIC, PROBABILISTIC};
	static const TrafficMode _traffic_mode=DETERMINISTIC;
	enum ArrivalMode{EXACT, THRESHOLD};
	const ArrivalMode _arrival_mode;

	std::list<std::shared_ptr<UAV> > generateTraffic(int step);
	
	bool atDestinationFix(const UAV &u);
	int ID;

	XY loc;
	double p_gen;
	int gen_rate;

private:
	// FOR PASSING IN TO UAVS
	TypeAStarAbstract* highPlanners;
	SectorAStarGrid* lowPlanners;
	vector<Fix>* fixes; // for generating destinations
	const double dist_thresh;
	
};