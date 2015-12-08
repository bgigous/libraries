#pragma once

#include <list>
#include <vector>
#include <memory>
#include "UAV.h"
#include "../../Planning/TypeAStarAbstract.h"
#include "../../projects/IROS2015/IROS2015/SectorAStarGrid.h"

class Fix{
public:
	Fix(XY loc, int ID, TypeAStarAbstract* planners, SectorAStarGrid* lowPlanners);
	~Fix(){};
	
	
	enum TrafficMode{DETERMINISTIC, PROBABILISTIC};
	static const TrafficMode _traffic_mode=DETERMINISTIC;
	enum ArrivalMode{EXACT, THRESHOLD};
	const ArrivalMode _arrival_mode;

	std::list<std::shared_ptr<UAV> > generateTraffic(std::vector<Fix>* fixes);
	
	bool atDestinationFix(const UAV &u);
	int ID;

	XY loc;

private:
	// FOR PASSING IN TO UAVS
	TypeAStarAbstract* highPlanners;
	SectorAStarGrid* lowPlanners;
	double p_gen;
	const double dist_thresh;
	
};