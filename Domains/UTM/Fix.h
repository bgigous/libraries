#pragma once

#include <list>
#include <vector>
#include <memory>
#include "UAV.h"
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"

class Fix{
public:
	Fix(easymath::XY loc, int ID, TypeGraphManager* highGraph, SectorGraphManager* lowGraph, vector<Fix>* fixes, UTMModes* params);
	~Fix(){};
	UTMModes* params;
	std::list<UAV_ptr> generateTraffic(int step);
	bool atDestinationFix(const UAV &u);
	int ID;
	easymath::XY loc;
	

private:
	TypeGraphManager* highGraph;
	SectorGraphManager* lowGraph;
	vector<Fix>* fixes; // for generating destinations
};