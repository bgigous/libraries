#include "UAV.h"


using namespace easymath;

UAV::UAV(XY start_loc, XY end_loc, UAVType t, AStarManager* planners):
	planners(planners),loc(start_loc), end_loc(end_loc), type_ID(t),speed(1.0)
{
	static int calls=0;
	ID = calls++;
	sectors_touched.insert(curSectorID());
};

std::list<int> UAV::getBestPath(int memstart, int memend){
	return planners->Astar_highlevel[type_ID]->search(memstart, memend);
}

void UAV::planAbstractPath(){
	sectors_touched.insert(curSectorID());
	list<int> high_path = planners->Astar_highlevel[type_ID]->search(curSectorID(), endSectorID());
	if (high_path_prev!=high_path){
		pathChanged=true;
		high_path_prev = high_path;
	} else {
		pathChanged=false;
	}
}

void UAV::planDetailPath(){
	int memstart =planners->getMembership(loc);
	int memend =  planners->getMembership(end_loc);
	list<int> high_path = getBestPath(memstart, memend);
	if (high_path.size()==0){
		printf("no path found!");
		system("pause");
	}
	high_path_prev = high_path;
	

	int memnext = nextSectorID();

	XY waypoint;
	if (memnext==memend){
		waypoint = end_loc;
	} else {
		waypoint = planners->Astar_highlevel[type_ID]->locations[memnext];
	}

	// TODO: VERIFY HERE THAT THE HIGH_PATH_BEGIN() IS THE NEXT MEMBER... NOT THE FIRST...
	// target the center of the sector, or the goal if it is reachable
	vector<XY> low_path = planners->m2astar[memstart][memnext]->m.get_solution_path(loc,waypoint);

	if (low_path.empty()) low_path.push_back(loc); // stay in place...

	while (target_waypoints.size()) target_waypoints.pop(); // clear the queue;
	for (std::vector<XY>::reverse_iterator i=low_path.rbegin(); i!=low_path.rend(); i++) target_waypoints.push(*i); // adds waypoints to visit
	target_waypoints.pop(); // remove CURRENT location from target	

}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinalDirection(loc-planners->Astar_highlevel[type_ID]->locations[nextSectorID()]);
}

/*  SEE IF WE CAN GET AWAY WITHOUT USING THIS?
void UAV::incrementDelay(){
	if (!high_path_prev.size()){ // new, put on time
		if (memstart==memend){
			t = 0;
		} else {
			t = (int)connectionTimes[*high_path.begin()][*std::next(high_path.begin())];
		}
	} else if (t<=0 && memstart==memend){ // on final leg; get to goal
		loc = end_loc;
	} else if (t<=0){ // middle leg; plan rest of path
		high_path.pop_front();
		loc = planners->agentLocs[int(high_path.front())]; // now in a new sector!
		if (high_path.size()>1 && *high_path.begin()!=*std::next(high_path.begin())){
			t = (int)connectionTimes[*high_path.begin()][*std::next(high_path.begin())];
		} else {
			t = 0;
		}
		//high_path_prev = high_path; //added
	} else {
		t--;
	}
}*/

void UAV::moveTowardNextWaypoint(){
	if (!target_waypoints.size()) return; // return if no waypoints
	for (int i=0; i<speed; i++){
		loc = target_waypoints.front();
		target_waypoints.pop();
	}
}