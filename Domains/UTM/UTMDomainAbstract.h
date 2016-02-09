#pragma once
#include <memory>

#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../Planning/TypeGraphManager.h"
#include "UAV.h"
#include "Sector.h"
#include "Link.h"
#include "Fix.h"
#include "../../FileIO/FileOut.h"

class UTMDomainAbstract :
	public IDomainStateful
{
public:
	UTMDomainAbstract(UTMModes* params=NULL);
	~UTMDomainAbstract(void);

	void synch_step(int* step_set){
		step = step_set;
	}

	UTMModes* params;
	UTMFileNames* filehandler;
	
	// Rewards
	matrix1d getDifferenceReward();
	double getGlobalReward();

	// Agents
	IAgentManager* agents;

	// Moving parts
	std::vector<Sector_ptr> sectors;
	std::vector<Link_ptr> links;
	std::vector<Fix_ptr> fixes;
	std::map<std::pair<int,int>,int> *linkIDs;


	// Traffic
	std::list<UAV_ptr> UAVs;
	void getNewUAVTraffic();
	void absorbUAVTraffic();

	TypeGraphManager* highGraph;

	// Base function overloads
	matrix2d getStates();
	matrix3d getTypeStates();
	void simulateStep(matrix2d agent_actions);
	void logStep();
	std::string createExperimentDirectory();

	// UAV motion tracking
	void logUAVLocations();
	matrix2d UAVLocations; // UAVLocation is nUAVs*2 x nSteps long, with the first dimension being twice as long because there are x- and y-values
	void exportUAVLocations(int fileID);
	void exportSectorLocations(int fileID);

	// Different from children
	virtual matrix1d getPerformance();
	virtual matrix1d getRewards();
	virtual void incrementUAVPath();
	virtual void detectConflicts();

	virtual void getPathPlans();
	virtual void getPathPlans(std::list<UAV_ptr> &new_UAVs);
	virtual void reset();


	//! Moves all it can in the list. Those eligible to move but who are blocked are left after the function.
	void try_to_move(std::vector<UAV_ptr> & eligible_to_move);
};