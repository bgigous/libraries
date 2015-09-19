#pragma once

#include <list>
#include "../../Planning/AStarManager.h"
#include "../../Math/easymath.h"
#include "../../Math/Matrix.h"


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
typedef Matrix<bool,2> barrier_grid;
typedef Matrix<int,2> ID_grid;

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	const enum UAVType{SLOW, FAST, NTYPES=5};
	//const enum UAVType{SLOW,NTYPES};

	UAV(easymath::XY start_loc, easymath::XY end_loc,
		std::vector<std::vector<XY> > *pathTraces, UAVType t, AStarManager* planners);

	~UAV(){};

	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over
	void pathPlan(bool abstraction_mode, map<int,map<int,double> > connection_times);
	AStarManager* planners; // shared with the simulator (for now);
	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc;
	easymath::XY end_loc;
	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level
	std::vector<std::vector<XY> > *pathTraces; // takes a pointer to the pathtrace for logging
	list<int> high_path_prev; // saves the high level path
	int nextSectorID(){
		return high_path_prev.front();
	}

	// ABSTRACTION MODE
	int t;
	int time_left_on_edge;
};