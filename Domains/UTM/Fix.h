#pragma once

#include "UAV.h"

class Fix;

class Fix{
public:
	typedef std::pair<int, int> edge;
	Fix(easymath::XY loc, int ID, TypeGraphManager* highGraph,
		std::vector<Fix*>* fixes, UTMModes* params, 
		std::map<edge,int> *linkIDs);


	~Fix(){};
	UTMModes* params;
	std::list<UAV*> generateTraffic(int step);
	virtual bool atDestinationFix(UAV &u);
	int ID;
	easymath::XY loc;
	std::map<edge,int>* linkIDs;
	virtual UAV* generate_UAV();

	TypeGraphManager* highGraph;
	std::vector<Fix*>* fixes; // for generating destinations
};

class FixDetail: public Fix{
public:
	FixDetail(easymath::XY loc, int ID, TypeGraphManager* highGraph, SectorGraphManager* lowGraph,
		std::vector<Fix*>* fixes, UTMModes* params, std::map<std::pair<int, int>, int> *linkIDs) :
		Fix(loc, ID, highGraph, fixes, params, linkIDs),
		lowGraph(lowGraph)
	{};
	~FixDetail() {};
	SectorGraphManager* lowGraph;
	virtual UAVDetail* generate_UAV();
	virtual bool atDestinationFix(UAVDetail &u);
};