#include "UAV.h"


using namespace easymath;
using namespace std;

UAV::UAV(XY start_loc, XY end_loc, UTMModes::UAVType t, TypeGraphManager* highGraph,
		 map<pair<int,int>,int>* linkIDs, UTMModes* params, SectorGraphManager* lowGraph):
	highGraph(highGraph),lowGraph(lowGraph),loc(start_loc), end_loc(end_loc), type_ID(t),speed(1.0), linkIDs(linkIDs),params(params)
{
	static int calls=0;
	ID = calls++;
	sectors_touched.insert(curSectorID());

	// Get initial plan and update
	planAbstractPath();
	update_link_info();

};


void UAV::update_link_info(){
	next_link_ID = nextLinkID();
	cur_link_ID = curLinkID();
}


int UAV::nextSectorID(int n){
	// Returns the nth sector ID from the current sector, or as far as it can go
	if (!high_path_prev.size())
		return curSectorID();

	int increment = (int)high_path_prev.size()>n?n:(high_path_prev.size()-1);
	return *std::next(high_path_prev.begin(),increment);
}

int UAV::curLinkID(){
	pair<int,int> link(curSectorID(), nextSectorID());
	if (link.first==link.second) return -1;
	else return linkIDs->at(link);
}

int UAV::curSectorID(){
	if (lowGraph==NULL)
		return highGraph->getMembership(loc); // return current sector
	else
		return lowGraph->getMembership(loc);
}

int UAV::endSectorID(){
	return highGraph->getMembership(end_loc);
}

int UAV::nextLinkID(){
	if (nextSectorID(1)==nextSectorID(2)) return curLinkID();
	else {
		pair<int,int> link(nextSectorID(1),nextSectorID(2));
		return linkIDs->at(link);
	}
}

std::list<int> UAV::getBestPath(int memstart, int memend){
	return highGraph->astar(memstart, memend, type_ID);
}

void UAV::planAbstractPath(){
	cur_link_ID = curLinkID();
	if (cur_link_ID!=-1) links_touched.insert(cur_link_ID);
	sectors_touched.insert(curSectorID());

	list<int> high_path;
	if (params->_search_type_mode==UTMModes::ASTAR){
		high_path = highGraph->astar(curSectorID(), endSectorID(), type_ID);
	} else {
		// RAGS
	}

	if (high_path_prev!=high_path){
		pathChanged=true;
		high_path_prev = high_path;
		update_link_info();

		if (lowGraph==NULL){
			clear(target_waypoints);
			for (int sector_id:high_path){
				target_waypoints.push(highGraph->getLocation(sector_id));
			}
		}
		update_link_info();
		// ERROR CONDITION: SHOULD NEVER BE HIT, FOR DEBUGGING
		if (high_path_prev.size()==1){
			printf("blah");
			highGraph->astar(curSectorID(),endSectorID(),type_ID);
			highGraph->print_graph("");
			exit(1);
		}
	} else {
		pathChanged=false;
	}
}

void UAV::planDetailPath(){
	// Get the high-level path
	high_path_prev = getBestPath(curSectorID(),endSectorID());

	// Get the astar low-level path
	XY next_loc = highGraph->getLocation(nextSectorID());
	vector<XY> low_path = lowGraph->astar(loc,next_loc);
	std::reverse(low_path.begin(),low_path.end());

	// Add to target waypoints
	clear(target_waypoints);
	for (XY i:low_path)
		target_waypoints.push(i);
	target_waypoints.pop(); // removes current location from target
}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinal_direction(loc-highGraph->getLocation(nextSectorID()));
}

void UAV::moveTowardNextWaypoint(){
	if (!target_waypoints.size())
		return; // return if no waypoints

	for (int i=0; i<speed; i++){
		loc = target_waypoints.front();
		target_waypoints.pop();
	}
}
