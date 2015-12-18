#pragma once

// STL includes
#include <utility>
#include <algorithm>

// Library includes
#include "UTMDomainAbstract.h"
#include "../../Planning/SectorGraphManager.h"


using namespace std;
using namespace easymath;

class UTMDomainDetail: 
	public UTMDomainAbstract
{
public:
	UTMDomainDetail();
	~UTMDomainDetail(void);

	// Base function overloads
	virtual matrix1d getRewards();
	virtual matrix1d getPerformance();
	virtual void getPathPlans(); // note: when is this event?
	virtual void getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs);
	virtual void exportLog(std::string fid, double G);
	virtual void detectConflicts();
	virtual void incrementUAVPath();
	virtual void reset();
	
	double conflict_thresh;
	
	// maps/Graph
	SectorGraphManager* lowGraph;
	void loadMaps();
	//Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later	//backend
	//std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]
	vector<XY> fix_locs;


	unsigned int getSector(easymath::XY p);
};

