#pragma once

#include <list>
#include <vector>
#include <memory>
#include "UAV.h"
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"

class Fix{
public:
	Fix(XY loc, int ID, TypeGraphManager* highGraph, SectorGraphManager* lowGraph, vector<Fix>* fixes, UTMModes* params);
	~Fix(){};
	UTMModes* params;
	std::list<std::shared_ptr<UAV> > generateTraffic(int step);
	bool atDestinationFix(const UAV &u);
	int ID;
	XY loc;
	

private:
	TypeGraphManager* highGraph;
	SectorGraphManager* lowGraph;
	vector<Fix>* fixes; // for generating destinations
};