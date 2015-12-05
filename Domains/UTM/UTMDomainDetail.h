#pragma once

// STL includes
#include <utility>
#include <algorithm>

// Library includes
#include "UTMDomainAbstract.h"


using namespace std;
using namespace easymath;

class UTMDomainDetail: public UTMDomainAbstract
{
public:
	UTMDomainDetail(vector<pair<int,int> > edges);
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
	
	// maps/planners
	void loadMaps();
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later	//backend
	std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]
	vector<XY> fix_locs;


	unsigned int getSector(easymath::XY p);

	// Conflict detection/logging
	Matrix<int,2> *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

};

