#pragma once
#include <memory>
#include <direct.h>

#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/TypeAStarAbstract.h"
#include "../../Math/Matrix.h"
#include "UAV.h"
#include "Sector.h"
#include "Fix.h"



// Output files
#define EXPERIMENT_FOLDER "Experiments/" // DIRECTORY HIERARCHY: EXPERIMENTS/NAGENTS/TRAFFIC/CAPACITY/TYPEHANDLING/REWARDTYPE(file name)
// CURRENTLY NOT TRYING DIFFERENT TYPES APPROACHES

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

	enum AirspaceMode{SAVED,GENERATED};
	AirspaceMode _airspace_mode;

	// Moving parts
	std::vector<Sector>* sectors;
	std::vector<Fix>* fixes;
	
	// Traffic
	std::list<std::shared_ptr<UAV> > UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	matrix2d sectorCapacity;
	matrix2d connectionTime;
	void getNewUAVTraffic(int step);
	void absorbUAVTraffic();
	
	TypeAStarAbstract* highPlanners;
	
	// Base function overloads
	matrix2d getStates();
	matrix3d getTypeStates();
	void simulateStep(matrix2d agent_actions);
	void logStep(int step);
	string createExperimentDirectory();

	// Different from children
	virtual matrix1d getPerformance();
	virtual matrix1d getRewards();
	virtual void incrementUAVPath();
	virtual void detectConflicts();
	virtual void getPathPlans();
	virtual void getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs);
	virtual void reset();

	int conflict_count;
	matrix1d conflict_minus_downstream; // conflict for entire system minus those downstream of agent i
	matrix1d conflict_minus_touched; // conflict for entire system minus those that touched agent i
	matrix1d conflict_random_reallocation; // conflict for entire system with agent i's traffic reallocated to others
	matrix1d conflict_node_average; // conflict with sector's conflict replaced by an average

	// File output
	string getRewardModeName(RewardMode mode);
};

