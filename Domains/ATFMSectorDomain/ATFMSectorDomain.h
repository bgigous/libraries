#pragma once

// STL includes
#include <utility>
#include <algorithm>

// Library includes
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/AStarManager.h"
#include "../../Math/Matrix.h"
#include "UAV.h"
#include "Sector.h"
#include "Fix.h"
#include <memory>

#define WORLD_SIZE 100.0


using namespace std;
using namespace easymath;

class ATFMSectorDomain: public IDomainStateful
{
public:

	ATFMSectorDomain(bool deterministic=false);
	~ATFMSectorDomain(void);

	// Base function overloads
	matrix1d getRewards();
	matrix1d getPerformance();
	matrix2d getStates();
	matrix3d getTypeStates();
	
	// SIM STATE VARIABLES
	bool is_deterministic; // the simulation is deterministic (for testing learning)
	

	// REWARD THINGS
	vector<Demographics> getLoads(); //SECTOR STUFF
	double G(vector<vector<int> > loads, vector<vector<int> > capacities); // SECTOR STUFF
	
	double conflict_thresh;
	
	// CORE DYNAMICS OBJECTS
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later


	unsigned int getSector(easymath::XY p);

	std::vector<Sector>* sectors;
	std::list<std::shared_ptr<UAV> > UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	std::vector<Fix>* fixes;

	void simulateStep(matrix2d agent_actions);
	void incrementUAVPath();

	void getNewUAVTraffic();
	void absorbUAVTraffic();
	void getPathPlans(); // note: when is this event?
	void getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs);

	void reset();
	void logStep(int step);
	void exportLog(std::string fid, double G);


	// PATH SNAPSHOT OUTPUT
	void pathSnapShot(int snapnum);
	void pathTraceSnapshot();
	vector<vector<XY> > *pathTraces; // [UAV_ID][step]

	// Conflict detection/logging
	void detectConflicts();
	int conflict_count;
	Matrix<int,2> *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

	//backend
	std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]
	vector<XY> agent_locs;
	
	AStarManager* planners;
	// for A* (boost) -- REPLACE THIS WITH ASTARMANAGER
	/*
	void setCostMaps(matrix2d agent_actions);
	void resetGraphWeights(matrix2d); // resets the weights on the graph and re-initializes graph
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	vector<AStar_easy::edge> edges;
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;
	void printMasks(); // ASTAR
	*/
};

