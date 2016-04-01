#pragma once

// STL includes
#include <utility>
#include <algorithm>

// Library includes
#include "UTMDomainAbstract.h"
#include "../../Planning/SectorGraphManager.h"
#include "../../FileIO/FileIn.h"


using namespace std;

class UTMDomainDetail:
	public UTMDomainAbstract
{
public:
	UTMDomainDetail(UTMModes* params_set);
	~UTMDomainDetail(void);

	// Base function overloads
	virtual matrix1d getRewards();
	virtual matrix1d getPerformance();
	virtual void getPathPlans(); // note: when is this event?
	virtual void getPathPlans(std::list<UAV*> &new_UAVs);
	virtual void exportLog(std::string fid, double G);
	virtual void detectConflicts();
	virtual void incrementUAVPath();
	virtual void reset();

	// maps/Graph
	SectorGraphManager* lowGraph;
	void loadMaps();
	//Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later	//backend
	//std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]
	vector<easymath::XY> fix_locs;

	void addConflict(UAV* u1, UAV* u2){
		agents->metrics.at(u1->curSectorID()).local[u1->type_ID]+=0.5;
		agents->metrics.at(u2->curSectorID()).local[u2->type_ID]+=0.5;
	}
	uint getSector(easymath::XY p);
};

