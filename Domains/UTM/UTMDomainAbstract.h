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


class UTMDomain :
	public IDomainStateful
{
public:
	UTMDomain(UTMModes* params);
	~UTMDomain() {};
	virtual void initialize(UTMModes* params) = 0;
};

class UTMDomainAbstract :
	public IDomainStateful
{
public:
	typedef std::pair<int, int> edge;
	UTMDomainAbstract(UTMModes* params);
	~UTMDomainAbstract(void);

//	virtual void initialize(UTMModes* params);

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
	std::map<edge,int> *linkIDs;


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
	matrix2d linkUAVs; // The number of UAVs on each link, [step][linkID]
	matrix2d sectorUAVs; // The number of UAVs waiting at a sector, [step][sectorID]
	void exportStepsOfTeam(int team, std::string suffix);
	std::string createExperimentDirectory();

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

private:
	matrix1d numUAVsAtSector; // records number of UAVs at each sector at current time step
};