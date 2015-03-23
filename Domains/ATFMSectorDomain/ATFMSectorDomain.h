#pragma once

// STL includes
#include <utility>

// Library includes
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../FileIO/easyio/easyio.h"
#include "../../Planning/AStar_easy.h"


#define WORLD_SIZE 100.0


using namespace std;
using namespace easymath;

class Sector;

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	const enum UAVType{SLOW, FAST, NTYPES};
	
	UAV(easymath::XY start_loc, easymath::XY end_loc,
		std::vector<std::vector<XY> > *pathTraces, UAVType t);

	~UAV(){};

	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over
	void pathPlan(AStar_easy* Astar_highlevel, vector<vector<bool> >*obstacle_map,
				   vector<vector<int> >* membership_map, vector<Sector>* sectors, 
				   map<list<AStar_easy::vertex>, AStar_easy* > &astar_lowlevel);
	
	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc;
	easymath::XY end_loc;
	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level
	std::vector<std::vector<XY> > *pathTraces; // takes a pointer to the pathtrace for logging
	list<AStar_easy::vertex> high_path_prev; // saves the high level path
};

class Fix{
public:
	Fix(XY loc);
	~Fix(){};
	
	std::list<UAV> generateTraffic(std::vector<Fix>* fixes, vector<vector<bool> >* obstacle_map,std::vector<std::vector<XY> > *pathTraces);
	void absorbTraffic(std::list<UAV>* UAVs);
	bool atDestinationFix(const UAV &u);

	XY loc;
	const double p_gen;
};

class Sector{
public:
	// An area of space that contains some fixes
	Sector(XY xy);
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center
};

class ATFMSectorDomain: public IDomainStateful
{
public:
	ATFMSectorDomain(void);
	~ATFMSectorDomain(void);

	// Base function overloads
	matrix1d getRewards();
	matrix1d getPerformance();
	matrix2d getStates();
	matrix3d getTypeStates();


	unsigned int getSector(easymath::XY p);

	std::vector<Sector>* sectors;
	std::list<UAV>* UAVs; // this is in a list because it has to be modified often. Never tie an ID/index to a UAV
	std::vector<Fix>* fixes;

	std::vector<std::vector<double> > cost_map; // cost matrix (Nx4), [n1,n2,cost,var]
	std::vector<std::vector<int> > direction_map; // direction (cardinal) needed to travel to go from [node1][node2]

	void simulateStep(std::vector<std::vector<double> > agent_actions);
	void incrementUAVPath();

	void setCostMaps(std::vector<std::vector<double> > agent_actions);
	void getNewUAVTraffic();
	void absorbUAVTraffic();
	void getPathPlans(); // note: when is this event?

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
	std::vector<std::vector<int> > *conflict_count_map; // this counts the number of conflicts in each grid (reset() clears this)

	//backend
	std::vector<std::vector<int> > * membership_map; // technically this should be an int matrix. fix later
	std::vector<std::vector<bool> > * obstacle_map; // pass these to uavs later to determine where the obstacles are
	//matrix2d* connection_map;


	// for A* (boost)
	vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	vector<double> weights;
	AStar_easy* Astar_highlevel;

	map<list<AStar_easy::vertex>, AStar_easy*> astar_lowlevel;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
};

