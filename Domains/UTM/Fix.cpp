#include "Fix.h"


using namespace easymath;
using namespace std;

Fix::Fix(XY loc, int ID_set, TypeGraphManager* highGraph, 
	vector<Fix*>* fixes, UTMModes* params, 
	map<edge,int> *linkIDs): 
	highGraph(highGraph), fixes(fixes), ID(ID_set),
	loc(loc), params(params),linkIDs(linkIDs)
{
}

bool Fix::atDestinationFix(UAV &u){
	return u.mem_end == u.mem;
}

bool FixDetail::atDestinationFix(UAVDetail &u) {
	if (params->_arrival_mode == UTMModes::ArrivalMode::EXACT)
		return u.end_loc == u.loc;
	else if (params->_arrival_mode == UTMModes::ArrivalMode::THRESHOLD) {
		return u.target_waypoints.size()				// UAV has planned a trajectory
			&& u.target_waypoints.front() == loc				// UAV wants to go there next
			&& easymath::euclidean_distance(u.loc, loc) < params->get_dist_thresh()	// UAV is close enough
			&& u.end_loc == loc;								// This is destination fix
	}
}

std::list<UAV* > Fix::generateTraffic(int step){
	// Creates a new UAV in the world
	std::list<UAV* > newTraffic;

	switch(params->_traffic_mode){
	case UTMModes::TrafficMode::PROBABILISTIC:
		if (double(rand())/double(RAND_MAX) > params->get_p_gen())	// Doesn't generate a UAV
			return newTraffic;
		break;
	case UTMModes::TrafficMode::DETERMINISTIC:
		if (step%params->get_gen_rate()!=0)	// Doesn't generate a UAV
			return newTraffic;
		break;
	default:
		return newTraffic;
	}

	UAV* u = generate_UAV();
	newTraffic.push_back(u);

	return newTraffic;
}

UAV* Fix::generate_UAV() {
	static int calls = 0;
	XY end_loc;
	if (ID == 0)
		end_loc = fixes->back()->loc;
	else
		end_loc = fixes->at(ID - 1)->loc; // go to previous

	UTMModes::UAVType type_id_set = UTMModes::UAVType(calls%int(params->get_n_types())); // EVEN TYPE NUMBER
	calls++;
	UAV* u = new UAV(highGraph->getMembership(loc), highGraph->getMembership(end_loc), type_id_set, highGraph, linkIDs, params);
	u->planAbstractPath();
	return u;
}

UAVDetail* FixDetail::generate_UAV() {
	static int calls = 0;
	XY end_loc;
	if (ID == 0)
		end_loc = fixes->back()->loc;
	else
		end_loc = fixes->at(ID - 1)->loc; // go to previous

	UTMModes::UAVType type_id_set = UTMModes::UAVType(calls%params->get_n_types()); // EVEN TYPE NUMBER
	UAVDetail* u = new UAVDetail(loc, end_loc, type_id_set, highGraph, linkIDs, params, lowGraph);
	u->planAbstractPath();
	return u;
}