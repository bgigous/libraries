#include "Fix.h"




Fix::Fix(XY loc, int ID_set, bool deterministic): 
	is_deterministic(deterministic), ID(ID_set), loc(loc), 
	//p_gen(0.05) // change to 1.0 if traffic controlled elsewhere
	p_gen(int(is_deterministic)*(1.0-0.05)+0.05), // modifies to depend on if deterministic
	dist_thresh(2.0)
{
}

bool Fix::atDestinationFix(const UAV &u){
	return u.end_loc == loc;


	/*
	return u.target_waypoints.size()					// UAV has planned a trajectory
	&& u.target_waypoints.front()==loc				// UAV wants to go there next
	&& easymath::distance(u.loc,loc)<dist_thresh	// UAV is close enough
	&& u.end_loc==loc;								// This is destination fix
	*/
}

std::list<UAV> Fix::generateTraffic(vector<Fix>* allFixes, barrier_grid* obstacle_map,std::vector<std::vector<XY> > *pathTraces){
	static int calls = 0;
	// Creates a new UAV in the world
	std::list<UAV> newTraffic;
	
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
		newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set));
	}



	/* VARIABLE TRAFFIC METHOD
	double coin = COIN_FLOOR0;
	if (coin<p_gen){
		XY end_loc;
		do {
			end_loc = allFixes->at(COIN_FLOOR0*allFixes->size()).loc;
		} while (end_loc==loc);
		UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES)); // EVEN TYPE NUMBER
		newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set));
	}*/

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