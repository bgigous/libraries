#pragma once

// STL includes
#include <utility>

// Library includes
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/AStar_easy.h"
#include "../../Planning/AStar_grid.h"
#include "../../Math/Matrix.h"
#include "UAV.h"
#include "Sector.h"
#include "Fix.h"


#define WORLD_SIZE 100.0


using namespace std;
using namespace easymath;

class ATFMSectorDomain: public IDomainStateful
{
public:
	vector<vector<int> > edge_time; // time that you need to be on an edge

	ATFMSectorDomain(bool deterministic=false);
	~ATFMSectorDomain(void);

	// Base function overloads
	matrix1d getRewards();
	matrix1d getPerformance();
	matrix2d getStates();
	matrix3d getTypeStates();
	vector<Demographics> getLoads();
	double G(vector<vector<int> > loads, vector<vector<int> > capacities);

	bool is_deterministic; // the simulation is deterministic (for testing learning)
	bool abstraction_mode; // in this mode, there is no low-level planning, and a simple network is used
	int connection_time[15][15]; // time to travel from first to second agent == HARDCODED FOR 15 AGENTS
	int connection_capacity[15][15][UAV::NTYPES]; // capacity for each type of UAV

	unsigned int getSector(easymath::XY p);

	std::vector<Sector>* sectors;
	std::list<UAV>* UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	std::vector<Fix>* fixes;

	std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]

	void simulateStep(matrix2d agent_actions);
	void incrementUAVPath();

	void setCostMaps(matrix2d agent_actions);
	void getNewUAVTraffic();
	void absorbUAVTraffic();
	void getPathPlans(); // note: when is this event?
	void getPathPlans(std::list<UAV> &new_UAVs);

	void reset();
	void logStep(int step);
	void exportLog(std::string fid, double G);

	void load_variable(Matrix<int,2> &var, std::string filename, std::string separator = STRING_UNINITIALIZED);
	void load_variable(Matrix<bool,2> &var, std::string filename, double thresh, std::string separator = STRING_UNINITIALIZED);
	void load_variable(std::vector<std::vector<bool> >* var, std::string filename, double thresh, std::string separator = STRING_UNINITIALIZED);


	// PATH SNAPSHOT OUTPUT
	void pathSnapShot(int snapnum);
	void pathTraceSnapshot();
	vector<vector<XY> > *pathTraces; // [UAV_ID][step]

	// Conflict detection/logging
	void detectConflicts();
	int conflict_count;
	Matrix<int,2> *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

	//backend
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	//matrix2d* connection_map;


	// for A* (boost) -- REPLACE THIS WITH ASTARMANAGER
	vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	// vector<double> weights; // old
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;

	void resetGraphWeights(matrix2d); // resets the weights on the graph and re-initializes graph

	//map<list<AStar_easy::vertex>, AStar_easy*> astar_lowlevel;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)


	void printMasks();
};

