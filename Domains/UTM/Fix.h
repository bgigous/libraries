#pragma once

#include "UAV.h"

class Fix;

class Fix{
public:
	Fix(easymath::XY loc, int ID, TypeGraphManager* highGraph, SectorGraphManager* lowGraph,
		std::vector<Fix*>* fixes, UTMModes* params, std::map<std::pair<int,int>,int> *linkIDs);


	~Fix(){};
	UTMModes* params;
	std::list<UAV*> generateTraffic(int step);
	bool atDestinationFix(UAV &u);
	int ID;
	easymath::XY loc;
	std::map<std::pair<int,int>,int>* linkIDs;
	

private:
	TypeGraphManager* highGraph;
	SectorGraphManager* lowGraph;
	std::vector<Fix*>* fixes; // for generating destinations
};