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
	UTMDomainAbstract(UTMModes* params);
	~UTMDomainAbstract(void);

	virtual void synch_step(int* step_set){
		step = step_set;
		agents->steps = step_set;
		printf("set the step\n");
	}
	
	UTMModes* params;
	UTMFileNames* filehandler;
	

	// Agents
	IAgentManager* agents;

	// Moving parts
	std::vector<Sector*> sectors;
	std::vector<Link*> links;
	std::vector<Fix*> fixes;
	std::map<std::pair<int,int>,int> *linkIDs;


	// Traffic
	std::list<UAV*> UAVs;
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
	virtual void getPathPlans(std::list<UAV*> &new_UAVs);
	virtual void reset();


	//! Moves all it can in the list. Those eligible to move but who are blocked are left after the function.
	void try_to_move(std::vector<UAV*> & eligible_to_move);

	// this has moved or something?
	//void move_UAV_to_link(UAV* u, Link* cur_link, Link* new_link); // handles motion of the UAV in the simulation, also includes logging
};