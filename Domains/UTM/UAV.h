#pragma once

//libraries includes
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"
#include "UTMModesAndFiles.h"

// STL includes
#include <queue>
#include <list>
#include <set>
#include <memory>

class UAV{
	/*
	This class is for moving UAVs in the airspace. They interact with the 
	environment through planning. Planning is done through boost. 
	*/
public:
	UAV(easymath::XY start_loc, easymath::XY end_loc, UTMModes::UAVType t, TypeGraphManager* highGraph, 
		std::map<std::pair<int,int>,int>* linkIDs, UTMModes* params, SectorGraphManager* lowGraph=NULL);

	~UAV(){
	}

	void set_cur_link_ID(int link_ID){
		cur_link_ID = link_ID;
	}

	UTMModes* params;

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
	std::list<int> high_path_prev; // saves the high level path
	std::map<std::pair<int,int>,int> *linkIDs;

	int next_link_ID;
	int cur_link_ID;
	
	//! Triggered when a UAV moves. This sets the 'next link ID' and 'current link ID' values
	//void update_link_info();

	//! Gets the sector ID from the location
	
	int nextSectorID(int n=1);
	
	//! Gets the link ID from the location and desired next loction
	int curLinkID();

	//! Gets the link ID of the next 'hop', or returns the current link if terminal
	int nextLinkID();
	
	//! Gets the sector ID from the location
	int curSectorID();

	//! Gets the sector ID for the desired end location
	int endSectorID();
	
	SectorGraphManager* lowGraph;

	// Delay modeling/abstraction mode
	int t;

	// Reward calculation stuff
	std::set<int> sectors_touched; // the sectors that the UAV has touched...
	std::set<int> links_touched; // the sectors that the UAV has touched...

	bool currently_in_conflict;
private:
	TypeGraphManager* highGraph; // shared with the simulator (for now);
};