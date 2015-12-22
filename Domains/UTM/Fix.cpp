#include "Fix.h"




Fix::Fix(XY loc, int ID_set, TypeGraphManager* highGraph, SectorGraphManager* lowGraph, vector<Fix>* fixes, UTMModes* params): 
	highGraph(highGraph), lowGraph(lowGraph), fixes(fixes), ID(ID_set), loc(loc), params(params)
{
}

bool Fix::atDestinationFix(const UAV &u){
	switch(params->_arrival_mode){
	case UTMModes::EXACT:
		return u.end_loc == u.loc;
		break;
	case UTMModes::THRESHOLD:
		return u.target_waypoints.size()				// UAV has planned a trajectory
		&& u.target_waypoints.front()==loc				// UAV wants to go there next
		&& easymath::distance(u.loc,loc)<params->get_dist_thresh()	// UAV is close enough
		&& u.end_loc==loc;								// This is destination fix
	default:
		printf("FATAL ERROR: No valid _arrival_mode chosen.");
		system("pause");
		exit(1);
	}
}

std::list<std::shared_ptr<UAV> > Fix::generateTraffic(int step){
	// Creates a new UAV in the world
	std::list<std::shared_ptr<UAV> > newTraffic;

	switch(params->_traffic_mode){
	case UTMModes::PROBABILISTIC:
		if (COIN_FLOOR0 > params->get_p_gen())	// Doesn't generate a UAV
			return newTraffic;
		break;
	case UTMModes::DETERMINISTIC:
		if (step%params->get_gen_rate()!=0)	// Doesn't generate a UAV
			return newTraffic;
		break;
	default:
		return newTraffic;
	}

	// Generates a UAV
	XY end_loc;
	if (ID==0)
		end_loc = fixes->back().loc;
	else
		end_loc = fixes->at(ID-1).loc; // go to previous

	UTMModes::UAVType type_id_set = UTMModes::UAVType(step%int(UTMModes::UAVType::NTYPES)); // EVEN TYPE NUMBER
	newTraffic.push_back(std::shared_ptr<UAV>(new UAV(loc,end_loc,type_id_set,highGraph,lowGraph)));

	return newTraffic;
}