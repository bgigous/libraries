#pragma once

//libraries includes
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"
#include "../../Math/easymath.h"
#include "UTMModesAndFiles.h"

// STL includes
#include <queue>
#include <list>

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	UAV(easymath::XY start_loc, easymath::XY end_loc, UTMModes::UAVType t, TypeGraphManager* highGraph, SectorGraphManager* lowGraph);

	~UAV(){}

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

	int next_link_ID;
	int cur_link_ID;
	
	//! Triggered when a UAV moves. This sets the 'next sector ID' and 'current sector ID' values
	void update_link_info(){
		next_link_ID = nextLinkID();
		cur_link_ID = curLinkID();
	}

	//! Gets the sector ID from the location
	int nextSectorID(int n=1){
		// Returns the nth sector ID from the current sector, or as far as it can go
		if (!high_path_prev.size())
			return curSectorID();
		
		int increment = high_path_prev.size()>n?n:(high_path_prev.size()-1);
		return *std::next(high_path_prev.begin(),increment);
	}

	//! Gets the link ID from the location and desired next loction
	int curLinkID(){
		pair<int,int> link(curSectorID(), nextSectorID());
		return highGraph->getEdgeID(link);
	}

	//! Gets the link ID of the next 'hop', or returns the current link if terminal
	int nextLinkID(){
		if (nextSectorID(1)==nextSectorID(2)) return curLinkID();
		else {
			pair<int,int> link(nextSectorID(1),nextSectorID(2));
			return highGraph->getEdgeID(link);
		}
	}

	//! Gets the sector ID from the location
	int curSectorID(){
		if (lowGraph==NULL)
			return highGraph->getMembership(loc); // return current sector
		else 
			return lowGraph->getMembership(loc);
	}

	//! Gets the sector ID for the desired end location
	int endSectorID(){
		return highGraph->getMembership(end_loc);
	}

	SectorGraphManager* lowGraph;

	// Delay modeling/abstraction mode
	int t;

	// Reward calculation stuff
	set<int> sectors_touched; // the sectors that the UAV has touched...
	set<int> links_touched; // the sectors that the UAV has touched...
private:
	TypeGraphManager* highGraph; // shared with the simulator (for now);
};


typedef std::shared_ptr<UAV> UAV_ptr;

static bool at_destination(UAV_ptr u){
	return  (u->loc==u->end_loc);
}