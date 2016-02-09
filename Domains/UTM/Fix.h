#pragma once

#include "UAV.h"

class Fix;
typedef std::shared_ptr<Fix> Fix_ptr;

class Fix{
public:
	Fix(easymath::XY loc, int ID, TypeGraphManager* highGraph, SectorGraphManager* lowGraph,
		std::vector<Fix_ptr>* fixes, UTMModes* params, std::map<std::pair<int,int>,int> *linkIDs);


	~Fix(){};
	UTMModes* params;
	std::list<UAV_ptr> generateTraffic(int step);
	bool atDestinationFix(const UAV &u);
	int ID;
	easymath::XY loc;
	std::map<std::pair<int,int>,int>* linkIDs;
	

private:
	TypeGraphManager* highGraph;
	SectorGraphManager* lowGraph;
	std::vector<Fix_ptr>* fixes; // for generating destinations
};