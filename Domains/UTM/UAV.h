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
	const enum UAVType{SLOW, FAST, NTYPES=1};
	//const enum UAVType{SLOW,NTYPES};

	UAV(easymath::XY start_loc, easymath::XY end_loc, UAVType t, AStarManager* planners);

	~UAV(){
		//printf("UAV %i dying.\n", ID);
		//system("pause");
	}
	
	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over


	void planAbstractPath();
	void planDetailPath();

	std::list<int> getBestPath(int memstart, int memene); // does not set anything within the UAV

	int ID;
	UAVType type_ID;
	double speed; // connected to type_ID
	easymath::XY loc;
	bool pathChanged;
	easymath::XY end_loc;
	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level
	list<int> high_path_prev; // saves the high level path
	int nextSectorID(){
		if (high_path_prev.size()>1){
			return *std::next(high_path_prev.begin()); // return second element (towards) of path
		} else {
			return planners->getMembership(loc); // return current sector
		}
	}

	int curSectorID(){
		return planners->getMembership(loc); // return current sector
	}
	int endSectorID(){
		return planners->getMembership(end_loc);
	}

	// ABSTRACTION MODE
	int t;
	set<int> sectors_touched; // the sectors that the UAV has touched...
private:
	AStarManager* planners; // shared with the simulator (for now);
};

	static bool at_destination(const std::shared_ptr<UAV> &u){
		if (u->loc==u->end_loc){
			//printf("UAV %i at dest\n",u->ID);
			return true;
		}
		return false;
	}