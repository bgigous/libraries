#pragma once

#include <list>
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"
#include "../../Math/easymath.h"
#include "../../Math/Matrix.h"
#include "UTMModesAndFiles.h"


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
	UAV(easymath::XY start_loc, easymath::XY end_loc, UTMModes::UAVType t, TypeGraphManager* highGraph, SectorGraphManager* lowGraph);

	~UAV(){
		/*printf("UAV %i dying.\n", ID);
		system("pause");*/
	}
	
	int getDirection(); // gets the cardinal direction of the UAV
	void moveTowardNextWaypoint(); // takes a time increment to move over


	void planAbstractPath();
	void planDetailPath();

	std::list<int> getBestPath(int memstart, int memend); // does not set anything within the UAV

	int ID;
	UTMModes::UAVType type_ID;
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
			return curSectorID(); // return current sector
		}
	}

	int curSectorID(){
		if (lowGraph==NULL)
			return highGraph->getMembership(loc); // return current sector
		else 
			return lowGraph->getMembership(loc);
	}
	int endSectorID(){
		return highGraph->getMembership(end_loc);
	}

	SectorGraphManager* lowGraph;
	// ABSTRACTION MODE
	int t;
	set<int> sectors_touched; // the sectors that the UAV has touched...
private:
	TypeGraphManager* highGraph; // shared with the simulator (for now);
};

	static bool at_destination(const std::shared_ptr<UAV> &u){
		return  (u->loc==u->end_loc);
	}