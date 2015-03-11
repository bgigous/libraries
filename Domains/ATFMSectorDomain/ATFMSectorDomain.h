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
	This class is for moving UAVs in the airspace. They interact with the environment through planning (OTHER SYSTEM), 
	*/
public:
	UAV(easymath::XY start_loc, easymath::XY end_loc,std::vector<std::vector<XY> > *pathTraces);
	easymath::XY loc;
	int ID;
	std::vector<std::vector<XY> > *pathTraces; // takes a pointer to the pathtrace for logging

	void pathPlan(AStar_easy* Astar_highlevel, vector<vector<bool> >*obstacle_map,
				   vector<vector<int> >* membership_map, vector<Sector>* sectors, 
				   map<list<AStar_easy::vertex>, AStar_easy* > &astar_lowlevel);

	std::queue<easymath::XY> target_waypoints; // target waypoints, low-level

	easymath::XY end_loc;
	double delay;
	list<AStar_easy::vertex> high_path_prev;


	int getDirection();
	void moveTowardNextWaypoint(); // takes a time increment to move over


	bool operator<(const UAV &p){
		return delay<p.delay;
	}

};

class Fix{
public:
	Fix(XY loc);
	XY loc;
	const double p_gen;

	std::list<UAV> generateTraffic(std::vector<Fix>* fixes, vector<vector<bool> >* obstacle_map,std::vector<std::vector<XY> > *pathTraces);
	void absorbTraffic(std::list<UAV>* UAVs);

	bool atDestinationFix(const UAV &u){
		double dist_thresh = 2.0;
		return u.target_waypoints.size()					// UAV has planned a trajectory
			&& u.target_waypoints.front()==loc				// UAV wants to go there next
			&& easymath::distance(u.loc,loc)<dist_thresh	// UAV is close enough
			&& u.end_loc==loc;								// This is destination fix
	}

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
	std::vector<double> getRewards();
	std::vector<double> getPerformance();

	std::vector<std::vector<double> > getStates();

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
	void pathSnapShot(int snapnum){
		matrix2d pathsnaps = matrix2d(2*UAVs->size());
		int ind = 0; // index of path given
		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			// pop through queue
			std::queue<XY> wpt_save = u->target_waypoints;
			while (u->target_waypoints.size()){
				pathsnaps[ind].push_back(u->target_waypoints.front().x);
				pathsnaps[ind+1].push_back(u->target_waypoints.front().y);
				u->target_waypoints.pop();
			}
			ind+=2;
		}
		PrintOut::toFile2D(pathsnaps,"path-" + to_string(snapnum)+".csv");
	}

	void pathTraceSnapshot(){
		// Exports all path traces
		matrix2d pathsnaps =matrix2d(2*pathTraces->size());
		for (int i=0, ind=0; i<pathTraces->size(); i++, ind+=2){
			for (int j=0; j<pathTraces->at(i).size(); j++){
				pathsnaps[ind].push_back(pathTraces->at(i)[j].x);
				pathsnaps[ind+1].push_back(pathTraces->at(i)[j].y);
			}
		}
		
		PrintOut::toFile2D(pathsnaps,"trace.csv");
	}

	vector<vector<XY> > *pathTraces; // [UAV_ID][step]


	// END PATH SNAPSHOT OUTPUT


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

