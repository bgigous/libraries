// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainDetail.h"
#include <string>
#include <list>
#include <vector>

using std::vector;
using easymath::XY;
using easymath::zeros;
using std::list;

UTMDomainDetail::UTMDomainDetail(UTMModes* params_set) :
    UTMDomainAbstract(params_set){
    //fix_locs(FileIn::read_pairs<XY>("agent_map/fixes.csv")) {
    // Add internal link IDs to the end of existing linkIDs
    // (important for internal travel)
    /*int cur_edge_num = linkIDs->size();
    for (int i = 0; i < sectors.size(); i++) {
        linkIDs->insert(make_pair(edge(i, i),cur_edge_num + i));
    }*/


    matrix2d membership_map =
        FileIn::read2<double>("agent_map/membership_map.csv");

    // Planning
    lowGraph = new SectorGraphManager(membership_map, highGraph->getEdges());

    // Get link IDs for fix generation
    // TODO: generate a fix somewhere in the airspace -- aim for the center, if not available pick random
    //for (size_t i = 0; i < fix_locs.size(); i++) {
    //    fixes.push_back(new FixDetail(fix_locs[i], i, highGraph, lowGraph,
    //        //&fixes, 
    //        fix_locs,
    //        params, linkIDs));
    //}
    vector<XY> sector_locs(sectors.size());
    for (int i = 0; i < sectors.size(); i++) {
        sector_locs[i] = sectors[i]->xy;
    }
    for (Sector* s : sectors) {
		delete s->generation_pt; // Delete the Fix created by the abstract constructor
        s->generation_pt = new FixDetail(s->xy, s->ID, highGraph, lowGraph, sector_locs, params, linkIDs);
    }

	// This is assuming there is one fix. If there's more and we need to keep track
	// of "done" UAVs, well... Guess we'll have to come up with something else
	for (int s = 0; s < params->n_sectors; s++)
		sectors[s]->generation_pt->UAVs_stationed = &UAVs_done[s];

    // NOTE: MAKE A 'SECTORDETAIL'?
}

UTMDomainDetail::~UTMDomainDetail(void) {
}


void UTMDomainDetail::logUAVLocations() {
    matrix1d stepLocation;
    for (UAV* u : UAVs) {
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.x);
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.y);
    }
    UAVLocations.push_back(stepLocation);
}


void UTMDomainDetail::exportUAVLocations(int fileID) {
    FileOut::print_vector(UAVLocations,
        "visualization/locations" + std::to_string(fileID) + ".csv");
}

vector<double> UTMDomainDetail::getPerformance() {
    return zeros(1);
    // return matrix1d(sectors.size(),-conflict_count);
}


vector<double> UTMDomainDetail::getRewards() {
    // MAY WANT TO ADD SWITCH HERE

    // DELAY REWARD
    return zeros(1);
    // return matrix1d(sectors.size(), -conflict_count);

    // LINEAR REWARD
    // return matrix1d(sectors->size(),-conflict_count);  // linear reward


    // QUADRATIC REWARD
    /* int conflict_sum = 0;
    for (int i=0; i<conflict_count_map->size(); i++){

    for (int j=0; j<conflict_count_map->at(i).size(); j++){
    int c = conflict_count_map->at(i)[j];
    conflict_sum += c*c;
    }
    }
    return matrix1d(sectors->size(),-conflict_sum);*/
}

size_t UTMDomainDetail::getSector(easymath::XY p) {
    // tests membership for sector, given a location
    return lowGraph->getMembership(p);
}

// HACK: ONLY GET PATH PLANS OF UAVS just generated
void UTMDomainDetail::getPathPlans() {
    // REPLACE WITH PLANPATH
    for (UAV* u : UAVs) {
		if (!static_cast<UAVDetail*>(u)->committed_to_link) // Indicates UAV has reached new sector and 
        // sets own next waypoint
	        static_cast<UAVDetail*>(u)->planDetailPath();
    }
}

void UTMDomainDetail::getPathPlans(const std::list<UAV* > &new_UAVs) {
    for (UAV* u : new_UAVs) {
		if (!static_cast<UAVDetail*>(u)->committed_to_link) // Indicates UAV has reached new sector and 
			static_cast<UAVDetail*>(u)->planDetailPath();  // sets own next waypoint
    }
}


void UTMDomainDetail::incrementUAVPath() {
    //for (UAV* u : UAVs) {
		// move toward next waypoint (next in low-level plan)
		//static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();

		//int nsID = u->nextSectorID(), csID = u->curSectorID();
		//if (nsID == csID)
		//{
		//	int n = u->next_link_ID;
		//	int c = u->cur_link_ID;
		//	if (n != c) // UAV has reached last link
		//		links[n]->move_from(u, links[c]);
		//	else
		//		u->mem = u->mem_end;
		//}
    //}

	for (size_t i = 0; i < links.size(); i++) {
		numUAVsOnLinks[i] = links[i]->traffic[0].size();
	}
	
	vector<UAV*> eligible;              // UAVs eligible to move to next link
	size_t num_agents = links.size() - sectors.size();
	copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [num_agents](UAV* u) {
		if (u->ID == 0)
			std::cout << std::endl;

		int csID = u->curSectorID();
		
		if (u->next_sector_ID == csID) { // UAV has reached boundary of the next sector
			if (u->cur_link_ID < num_agents && u->next_link_ID >= num_agents) { // internal link
				u->mem = u->mem_end;    // At destination. Moves to end of link.
				// since UAV is in goal sector, it is committed to the internal link and can move within it
				// but in order for it to be added to the internal link, we'll uncommit it, k?
				// It's guaranteed to be added since capacity is inf
				static_cast<UAVDetail*>(u)->committed_to_link = false;
				if (u->ID == 0)
					std::cout << std::endl;
				//static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
				return true;
			}
			else if (u->cur_link_ID >= num_agents)
			{
				// UAV is traveling through goal sector via its internal link
				return false;
			}
			else {
				// UAV has reached the next non-goal sector. It may have to wait before it can move,
				//	so it is free to choose a different link if necessary
				static_cast<UAVDetail*>(u)->committed_to_link = false;
				static_cast<UAVDetail*>(u)->planDetailPath(); // a no-no?
				return true;            // At end of non-destination link
			}
		}
		else {
			// TYPE IMPLEMENTATION
			// Not yet has it reached next sector
			// move toward next waypoint (next in low-level plan)			
			//static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
			if (!static_cast<UAVDetail*>(u)->committed_to_link)
				return true; // UAV got to a new sector, but is waiting to traverse next link
			else
				return false; // UAV has already committed to a link and is therefore not waiting
		}
	});

	if (eligible.empty()) {
		return;
	}
	else {
		// preemtively commit all eligible UAVs to link
		// those that don't move will uncommit (how rude)
		//for (UAV * u : eligible)
		//	static_cast<UAVDetail*>(u)->committed_to_link = true;
		// This moves all UAVs that are eligible and not blocked
		try_to_move(&eligible);
		// Only those that cannot move are left in eligible
		// std::printf("%i UAVs delayed. \n",eligible.size());

		for (UAV* u : eligible) {
			static_cast<UAVDetail*>(u)->committed_to_link = false;	// UAV is still waiting to move
																	// it can decide to take a different link before it moves
			// adds delay for each eligible UAV not able to move
			agents->add_delay(u);

			// Add 1 to the sector that the UAV is trying to move from
			// Carrie! Would this be correct, do ya think?
			// Different from AbstractDomain
			int n = u->curSectorID(); 
			numUAVsAtSector[n]++;

			// counterfactuals
			if (params->_reward_mode == UTMModes::RewardMode::DIFFERENCE_AVG)
				agents->add_average_counterfactual();
			else if (params->_reward_mode ==
				UTMModes::RewardMode::DIFFERENCE_DOWNSTREAM)
				agents->add_downstream_delay_counterfactual(u);
			else
				continue;
		}
	}

	for (UAV* u : UAVs)
		static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
}

void UTMDomainDetail::try_to_move(vector<UAV*> * eligible_to_move) {
	random_shuffle(eligible_to_move->begin(), eligible_to_move->end());

	size_t el_size;
	do {
		el_size = eligible_to_move->size();

		vector<Link*> L = links;
		eligible_to_move->erase(
			remove_if(eligible_to_move->begin(), eligible_to_move->end(),
				[L](UAV* u) {
			if (u->ID == 0)
				std::cout << std::endl;
			if (u->ID == 1)
				std::cout << std::endl;
			int n = u->next_link_ID;
			int c = u->cur_link_ID;
			int t = u->type_ID;
			// Carrie! I had to add the first condition because otherwise the UAVs
			// would be moved to the next link before they get to the right sector
			// Suggestions highly welcome if this is hacky
			if (!L[n]->at_capacity(t)) {
				L[n]->move_from(u, L[c]);
				static_cast<UAVDetail*>(u)->committed_to_link = true;
				return true;
			}
			else {
				return false;
			} }),
			eligible_to_move->end());
	} while (el_size != eligible_to_move->size());
}

void UTMDomainDetail::absorbUAVTraffic() {
	// Deletes UAVs
	vector<Link*> l = links;
	vector<Sector*> S = sectors;
	bool keep = params->_disposal_mode == UTMModes::DisposalMode::KEEP ? true : false;
	if (keep) { // remove UAVs from the domain, but keep track of where they were for later
		// THIS IS ASSUMING 1 FIX PER SECTOR
		for (int s = 0; s < params->n_sectors; s++)
		{
			UAVs_done[s].splice(UAVs_done[s].begin(), UAVs, remove_if(UAVs.begin(), UAVs.end(), [l, S, s](UAV* u) {
				if (u->mem == u->mem_end && u->curSectorID() == s) {
					FixDetail* fix = (FixDetail*)S[u->curSectorID()]->generation_pt;
					if (fix->atDestinationFix(*static_cast<UAVDetail*>(u))) {
						l[u->cur_link_ID]->remove(u);
						static_cast<UAVDetail*>(u)->committed_to_link = false; // uncommit it since it's done anyway
						return true;
					}
					else {
						return false;
					}
				}
				return false;
			}), UAVs.end());
		}
	}
	else { // just get rid of UAVs forever if the reach their respective goals
		UAVs.erase(remove_if(UAVs.begin(), UAVs.end(), [l, S](UAV* u) {
			if (u->mem == u->mem_end) {
				FixDetail* fix = (FixDetail*)S[u->curSectorID()]->generation_pt;
				if (fix->atDestinationFix(*static_cast<UAVDetail*>(u))) {
					l[u->cur_link_ID]->remove(u);
					delete u;
					return true;
				}
				else {
					return false;
				}
			}
			return false;
		}), UAVs.end());
	}
}

void UTMDomainDetail::getNewUAVTraffic() {
	// Generates (with some probability) plane traffic for each sector
	for (Sector* s : sectors) {
		list<UAV*> new_UAVs = s->generation_pt->generateTraffic(*step);
		for (UAV* u : new_UAVs) {
			UAVs.push_back(u);
			links.at(u->cur_link_ID)->add(u);
			if (!links.at(u->cur_link_ID)->at_capacity(u->type_ID)) {
				static_cast<UAVDetail*>(u)->committed_to_link = true;
				static_cast<UAVDetail*>(u)->planDetailPath(); // I don't know if it's a no-no to do it here
															// 
			}
		}
	}
}

void UTMDomainDetail::reset() {
    UAVs.clear();
    UAVLocations.clear();
}


void UTMDomainDetail::exportLog(std::string fid, double ) {
    static int calls = 0;
    calls++;
}

void UTMDomainDetail::detectConflicts() {
    for (list<UAV* >::iterator u1 = UAVs.begin(); u1 != UAVs.end(); ++u1) {
        for (list<UAV* >::iterator u2 = std::next(u1); u2 != UAVs.end(); ++u2) {
            XY a = static_cast<UAVDetail*>(*u1)->loc;
            XY b = static_cast<UAVDetail*>(*u2)->loc;
            double d = easymath::euclidean_distance(a, b);

            if (d > params->get_conflict_thresh()) continue;  // No conflict!

            addConflict(*u1, *u2);
        }
    }
}
