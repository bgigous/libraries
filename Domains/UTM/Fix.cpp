#include "Fix.h"




Fix::Fix(XY loc, int ID_set, AStarManager* planners): 
	planners(planners), ID(ID_set), loc(loc),
	_arrival_mode(EXACT), dist_thresh(2.0)
{
	switch (_traffic_mode){
	case DETERMINISTIC:
		p_gen = 1.0;
		break;
	case PROBABILISTIC:
		p_gen = 0.5;
		break;
	default:
		printf("No valid _traffic_mode chosen.");
		system("pause");
	}
}

bool Fix::atDestinationFix(const UAV &u){
	switch(_arrival_mode){
	case EXACT:
		return u.end_loc == u.loc;
		break;
	case THRESHOLD:
		return u.target_waypoints.size()				// UAV has planned a trajectory
		&& u.target_waypoints.front()==loc				// UAV wants to go there next
		&& easymath::distance(u.loc,loc)<dist_thresh	// UAV is close enough
		&& u.end_loc==loc;								// This is destination fix
	default:
		printf("No valid _arrival_mode chosen.");
		system("pause");
	}
}

std::list<std::shared_ptr<UAV> > Fix::generateTraffic(vector<Fix>* allFixes){
	static int calls = 0;
	// Creates a new UAV in the world
	std::list<std::shared_ptr<UAV> > newTraffic;
	
	// CONSTANT TRAFFIC FLOW METHOD
	double coin = COIN_FLOOR0;
	if (coin<p_gen){
		XY end_loc;
		if (ID==0){
			end_loc = allFixes->back().loc;
		} else {
			end_loc = allFixes->at(ID-1).loc; // go to previous
		}
		UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES)); // EVEN TYPE NUMBER
		newTraffic.push_back(std::shared_ptr<UAV>(new UAV(loc,end_loc,type_id_set,planners)));
	}

	calls++;
	return newTraffic;
}

void Fix::absorbTraffic(list<UAV>* UAVs){
	// INEFFICIENT: MOVED OUT OF FIX FUNCTION

	/*
	list<UAV> cur_UAVs;
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		if (atDestinationFix(*u)){
			// don't copy over
		} else {
			if (u->target_waypoints.size() && u->target_waypoints.front()==loc){ // deleted if size==0; drop invalid plans
				u->target_waypoints.pop();
			}
			cur_UAVs.push_back(*u);
		}
	}
	(*UAVs) = cur_UAVs; // copy over
	*/
}