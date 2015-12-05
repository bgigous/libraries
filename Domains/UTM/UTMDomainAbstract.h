#pragma once
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/AStarManager.h"
#include "../../Math/Matrix.h"
#include "../../projects/IROS2015/IROS2015/AirspaceMap.h"
#include "UAV.h"
#include "Sector.h"
#include "Fix.h"
#include <memory>

class UTMDomainAbstract :
	public IDomainStateful
{
public:
	UTMDomainAbstract();
	~UTMDomainAbstract(void);

	static const enum RewardMode{GLOBAL, DIFFERENCE_DOWNSTREAM,DIFFERENCE_TOUCHED,DIFFERENCE_REALLOC,DIFFERENCE_AVG, NMODES};
	RewardMode _reward_mode;
	matrix1d getDifferenceReward();
	double getGlobalReward();
	matrix1d getLocalReward();

	AirspaceMap* airspace;
	enum AirspaceMode{SAVED,GENERATED};
	AirspaceMode _airspace_mode;

	// Moving parts
	std::vector<Sector>* sectors;
	std::vector<Fix>* fixes;
	
	// Traffic
	std::list<std::shared_ptr<UAV> > UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	void getNewUAVTraffic();
	void absorbUAVTraffic();
	
	AStarManager* planners;
	
	// Base function overloads
	matrix2d getStates();
	matrix3d getTypeStates();
	void simulateStep(matrix2d agent_actions);
	void logStep(int step);

	// Different from children
	virtual matrix1d getPerformance();
	virtual matrix1d getRewards();
	virtual void incrementUAVPath();
	virtual void detectConflicts();
	virtual void getPathPlans();
	virtual void getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs);
	virtual void exportLog(std::string fid, double G);
	virtual void reset();

	int conflict_count;
	matrix1d conflict_minus_downstream; // conflict for entire system minus those downstream of agent i
	matrix1d conflict_minus_touched; // conflict for entire system minus those that touched agent i
	matrix1d conflict_random_reallocation; // conflict for entire system with agent i's traffic reallocated to others
	matrix1d conflict_node_average; // conflict with sector's conflict replaced by an average
};

