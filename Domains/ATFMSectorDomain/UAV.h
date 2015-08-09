#pragma once

#include <list>
#include "../../Planning/AStar_easy.h"
#include "../../Math/easymath.h"
#include "../../Math/Matrix.h"
#include "../../Planning/AStar_grid.h"


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
typedef Matrix<bool,2> barrier_grid;
typedef Matrix<int,2> ID_grid;
typedef std::map<int,std::map<int,AStar_grid*> > grid_lookup;

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	int time_left_on_edge;
	const enum UAVType{SLOW, FAST, NTYPES=5};
	//const enum UAVType{SLOW,NTYPES};

	UAV(easymath::XY start_loc, easymath::XY end_loc,
		std::vector<std::vector<XY> > *pathTraces, UAVType t);

	~UAV(){};

	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over
	void pathPlan(AStar_easy* Astar_highlevel, grid_lookup &m2astar, barrier_grid*obstacle_map,
		ID_grid* membership_map, std::vector<easymath::XY> sectorLocations, bool abstraction_mode, int connection_time[15][15]);

	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc;
	easymath::XY end_loc;
	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level
	std::vector<std::vector<XY> > *pathTraces; // takes a pointer to the pathtrace for logging
	list<AStar_easy::vertex> high_path_prev; // saves the high level path

	// ABSTRACTION MODE; TIME UNTIL SWITCH
	int t;
};
