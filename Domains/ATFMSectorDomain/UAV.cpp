#include "UAV.h"


using namespace easymath;

UAV::UAV(XY start_loc, XY end_loc,std::vector<std::vector<XY> > *pathTraces, UAVType t, AStarManager* planners):
	planners(planners),loc(start_loc), end_loc(end_loc), ID(pathTraces->size()), pathTraces(pathTraces), type_ID(t)
{
	pathTraces->push_back(vector<XY>(1,loc)); // new path trace created for each UAV,starting at location

	switch(t){
	case SLOW:
		speed = 1;
		break;
	case FAST:
		// substitute: high priority
		speed = 1;
		//speed = 2;
		break;
	default:{
		//printf("no speed found!!");
		speed = 1;
		//system("pause");
		break;
			}
	}

};

void UAV::pathPlan(bool abstraction_mode, map<int,map<int,double> > connection_times){
	list<int> high_path = planners->search(type_ID,loc,end_loc);
	int memstart = planners->getMembership(loc);
	int memend = planners->getMembership(end_loc);

	if (abstraction_mode){
		if (!high_path_prev.size()){ // new, put on time
			if (memstart==memend){
				t = 0;
			} else {
				t = connection_times[*high_path.begin()][*std::next(high_path.begin())];
			}
			high_path_prev = high_path;
		} else if (t<=0 && memstart==memend){ // on final leg; get to goal
			loc = end_loc;
		} else if (t<=0){ // middle leg; plan rest of path
			high_path.pop_front();
			loc = planners->agent_locs[int(high_path.front())]; // now in a new sector!
			if (high_path.size()>1 && *high_path.begin()!=*std::next(high_path.begin())){
				t = connection_times[*high_path.begin()][*std::next(high_path.begin())];
			} else {
				t = 0;
			} 
		} else {
			t--;
		}

	} else {

		if (high_path_prev != high_path){ // Check if any course change necessary
			high_path_prev = high_path;


			int memnext;
			if (high_path.size()==1){
				memnext = high_path.front();
			} else {
				memnext = *std::next(high_path.begin());
			}
			XY waypoint;
			if (memnext==memend){
				waypoint = end_loc;
			} else {
				waypoint = planners->agent_locs[memnext];
			}

			// TODO: VERIFY HERE THAT THE HIGH_PATH_BEGIN() IS THE NEXT MEMBER... NOT THE FIRST...
			// target the center of the sector, or the goal if it is reachable
			vector<XY> low_path = planners->m2astar[memstart][memnext]->m.get_solution_path(loc,waypoint);

			if (low_path.empty()) low_path.push_back(loc); // stay in place...

			while (target_waypoints.size()) target_waypoints.pop(); // clear the queue;
			for (int i=0; i<low_path.size(); i++) target_waypoints.push(low_path[i]); // adds waypoints to visit
			target_waypoints.pop(); // remove CURRENT location from target	
		}
	}
}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinalDirection(loc-end_loc);
}