#pragma once
#include <memory>

#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/TypeGraphManager.h"
#include "../../Math/Matrix.h"
#include "UAV.h"
#include "Sector.h"
#include "Fix.h"

typedef std::shared_ptr<UAV> UAV_ptr;

class UTMDomainAbstract :
	public IDomainStateful
{
public:
	UTMDomainAbstract(UTMModes* params=NULL, UTMFileNames* filehandler=NULL);
	~UTMDomainAbstract(void);

	UTMModes* params;
	UTMFileNames* filehandler;

	matrix1d getDifferenceReward();
	double getGlobalReward();
	matrix1d getLocalReward();


	// Moving parts
	std::vector<Sector>* sectors;
	std::vector<Fix>* fixes;
	
	// Traffic
	std::list<UAV_ptr> UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	matrix2d sectorCapacity;
	matrix2d connectionTime;
	void getNewUAVTraffic(int step);
	void absorbUAVTraffic();
	
	TypeGraphManager* highGraph;
	
	// Base function overloads
	matrix2d getStates();
	matrix3d getTypeStates();
	void simulateStep(matrix2d agent_actions, int step);
	void logStep(int step);
	string createExperimentDirectory();

	// UAV motion tracking
	void logUAVLocations(){
		matrix1d stepLocation;
		for (UAV_ptr u: UAVs){
			stepLocation.push_back(u->loc.x);
			stepLocation.push_back(u->loc.y);
		}
		UAVLocations.push_back(stepLocation);
	}
	matrix2d UAVLocations; // UAVLocation is nUAVs*2 x nSteps long, with the first dimension being twice as long because there are x- and y-values
	void exportUAVLocations(int fileID){
		FileOut::print2D(UAVLocations,"visualization/locations"+to_string(fileID)+".csv");
	}
	void exportAgentLocations(int fileID){
		matrix1d sectorLocations;
		for (Sector s: *sectors){
			sectorLocations.push_back(s.xy.x);
			sectorLocations.push_back(s.xy.y);
		}
		FileOut::print1D(sectorLocations,"visualization/agent_locations"+to_string(fileID)+".csv");
	}

	matrix3d agentActions;
	void logAgentActions(matrix2d agentStepActions){
		agentActions.push_back(agentStepActions);
	}
	void exportAgentActions(int fileID){
		FileOut::print3D(agentActions,"visualization/actions"+to_string(fileID)+".csv");
	}

	// Different from children
	virtual matrix1d getPerformance();
	virtual matrix1d getRewards();
	virtual void incrementUAVPath();
	virtual void detectConflicts();
	virtual void getPathPlans();
	virtual void getPathPlans(std::list<UAV_ptr> &new_UAVs);
	virtual void reset();

	int conflict_count;
	matrix1d conflict_minus_downstream; // conflict for entire system minus those downstream of agent i
	matrix1d conflict_minus_touched; // conflict for entire system minus those that touched agent i
	matrix1d conflict_random_reallocation; // conflict for entire system with agent i's traffic reallocated to others
	matrix1d conflict_node_average; // conflict with sector's conflict replaced by an average
};

