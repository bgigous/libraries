#include "UTMDomainAbstract.h"

using namespace std;
using namespace easymath;


UTMDomain::UTMDomain(UTMModes* params) :
	IDomainStateful(params)
{
};

/*UTMDomainAbstract::UTMDomainAbstract(UTMModes* params_set):
	UTMDomain(params_set)
{
	initialize(params);
}*/

UTMDomainAbstract::UTMDomainAbstract(UTMModes* params_set) :
	IDomainStateful(params_set)
{
	filehandler = new UTMFileNames(params_set),
		params = params_set;

	// Airspace construction
	int n_sectors;
	std::string domain_dir = filehandler->createDomainDirectory();
	ifstream edgefile(domain_dir + "edges.csv");
	bool fileExists = edgefile.good();
	edgefile.close();
	if (params->_airspace_mode == UTMModes::AirspaceMode::SAVED && fileExists) {
		highGraph = new TypeGraphManager(domain_dir + "edges.csv", domain_dir + "nodes.csv", n_types);
		n_sectors = highGraph->getNVertices();
	}
	else {
		// Generate a new airspace
		n_sectors = params->get_n_sectors();
		highGraph = new TypeGraphManager(n_sectors, n_types, 200.0, 2000);
		highGraph->print_graph(domain_dir); // saves the graph
	}
	params->n_links = highGraph->getEdges().size(); // Must be set after graph created
	n_agents = params->get_n_agents();

	// Link construction
	linkIDs = new map<edge, int>();
	vector<vector<int> > connections(n_sectors);
	for (edge e : highGraph->getEdges()) {
		int source = e.first; // membership of origin of edge
		int target = e.second; // membership of connected node
		XY source_loc = highGraph->getLocation(source);
		XY target_loc = highGraph->getLocation(target);
		int cardinal_dir = cardinal_direction(source_loc - target_loc);


		links.push_back(
			new Link(links.size(), source, target,
				int(manhattan_distance(source_loc, target_loc) / 10.0),
				vector<size_t>(n_types, size_t(params->get_flat_capacity())), cardinal_dir));
		linkIDs->insert(make_pair((make_pair(source, target)), links.size() - 1));

		connections[source].push_back(target);
	}

	// Sector construction
	for (int i = 0; i < n_sectors; i++)
		sectors.push_back(new Sector(highGraph->getLocation(i), i, n_sectors, connections[i], params));


	if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) agents = new SectorAgentManager(links, n_types, sectors, params);
	else agents = new LinkAgentManager(links.size(), n_types, links, params);

	// Fix construction
	for (Sector* s : sectors)
		fixes.push_back(new Fix(s->xy, s->ID, highGraph, &fixes, params, linkIDs));
}

string UTMDomainAbstract::createExperimentDirectory() {
	return filehandler->createExperimentDirectory();
}

UTMDomainAbstract::~UTMDomainAbstract(void)
{
	delete linkIDs;
	delete filehandler;
	delete highGraph;
	delete agents;

	for (Link* l : links)
		delete l;

	for (Sector* s : sectors)
		delete s;

	for (Fix* f : fixes)
		delete f;

	for (UAV* u : UAVs)
		delete u;
}

matrix1d UTMDomainAbstract::getPerformance() {
	return agents->performance();
}

matrix1d UTMDomainAbstract::getRewards() {
	return agents->reward();
}


void UTMDomainAbstract::incrementUAVPath() {
	vector<UAV*> eligible;				// UAVs eligible to move to the next link
	copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [](UAV* u) {
		if (u->t <= 0) {
			if (u->nextLinkID() == u->cur_link_ID) {
				u->mem = u->mem_end;	// At destination. Moves to end of link.
				return false;
			}
			else return true;			// At end of non-destination link
		}
		else {
			for (size_t i = 0; i <= u->type_ID; i++)	// TYPE IMPLEMENTATION
				u->t--;						// Not yet at end of link. Decrement time.
			return false;
		}
	});

	if (eligible.empty()) {
		return;
	}

	// CURRENTLY CONFLICTS MODE MAY NOT WORK...
	//else if (params->_reward_type_mode == UTMModes::RewardType::CONFLICTS) {
	//	for (UAV* u : eligible) {
	//		u->high_path_prev.pop_front();
	//		u->loc = sectors[u->high_path_prev.front()]->xy;
	//	}
	//	return;
	//}
	else {
		try_to_move(eligible); // This moves all UAVs that are eligible and not blocked
		// Only those that cannot move are left in eligible
		//std::printf("%i UAVs delayed. \n",eligible.size());
		for (UAV* u : eligible) {
			agents->add_delay(u);	// adds delay for each eligible UAV not able to move

			// counterfactuals
			if (params->_reward_mode == UTMModes::RewardMode::DIFFERENCE_AVG)
				agents->add_average_counterfactual();
			else if (params->_reward_mode == UTMModes::RewardMode::DIFFERENCE_DOWNSTREAM)
				agents->add_downstream_delay_counterfactual(u);
			else continue;
		}
	}
}

void UTMDomainAbstract::try_to_move(vector<UAV*> & eligible_to_move) {
	random_shuffle(eligible_to_move.begin(), eligible_to_move.end());

	int el_size;
	do {
		el_size = eligible_to_move.size();

		vector<Link*> L = links;
		eligible_to_move.erase(
			remove_if(eligible_to_move.begin(), eligible_to_move.end(),
				[L](UAV* u) {
			int n = u->next_link_ID;
			int c = u->cur_link_ID;
			int t = u->type_ID;
			if (!L[n]->at_capacity(t)) {
				L[n]->move_from(u, L[c]);
				return true;
			}
			else {
				return false;
			}
		}
		), eligible_to_move.end());

	} while (el_size != eligible_to_move.size());
}

matrix2d UTMDomainAbstract::getStates() {
	matrix2d allStates(n_agents, matrix1d(n_state_elements, 0.0));

	/* "NORMAL POLARITY" state
	for (UAV* u : UAVs){
		allStates[getSector(u->loc)][u->getDirection()]+=1.0; // Adds the UAV impact on the state
	}
	*/

	// REVERSED POLARITY STATE
	/*for (UAV* u : UAVs){
		// COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
		if (u->nextSectorID()==u->curSectorID()) continue;
		else allStates[u->nextSectorID()][u->getDirection()] += 1.0;
	}*/

	// CONGESTION STATE

	if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
		vector<int> sector_congestion_count(n_agents, 0);
		for (UAV* u : UAVs) {
			sector_congestion_count[u->curSectorID()]++;
		}
		for (uint i = 0; i < sectors.size(); i++) {
			for (int conn : sectors[i]->connections) {

				allStates[i][cardinal_direction(sectors[i]->xy - sectors[conn]->xy)] += sector_congestion_count[conn];
			}
		}
	}
	else {
		for (UAV* u : UAVs)
			allStates[u->cur_link_ID][u->type_ID]++;
	}


	agents->agentStates.push_back(allStates);

	return allStates;
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions) {
	// Alter the cost maps (agent actions)
	agents->logAgentActions(agent_actions);
	bool action_changed = agents->last_action_different();

	// New UAVs appear
	getNewUAVTraffic();

	if (action_changed)
		highGraph->setCostMaps(agents->actions2weights(agent_actions));

	// Make UAVs reach their destination
	absorbUAVTraffic();

	// Plan over new cost maps
	if (action_changed)
		getPathPlans();

	// UAVs move
	incrementUAVPath();
	if (params->_reward_type_mode == UTMModes::RewardType::CONFLICTS)
		detectConflicts();

}

matrix3d UTMDomainAbstract::getTypeStates() {
	matrix3d allStates = zeros(n_agents, n_types, n_state_elements);

	matrix2d state_printout = zeros(n_agents, n_state_elements);
	if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
		for (UAV* u : UAVs) {
			int a = u->curSectorID();
			int id = u->type_ID;
			int dir = u->getDirection();
			allStates[a][id][dir] += 1.0;
			state_printout[a][dir]++;
		}
	}
	else {
		// link definition
		for (UAV* u : UAVs) {
			int a = u->cur_link_ID;
			int id = u->type_ID;
			allStates[a][id][0] += 1.0;
		}
	}
	agents->agentStates.push_back(state_printout);
	return allStates;
}

void UTMDomainAbstract::exportSectorLocations(int fileID) {
	std::vector<easymath::XY> sectorLocations;
	for (Sector* s : sectors)
		sectorLocations.push_back(s->xy);
	FileOut::print_pair_container(sectorLocations, "visualization/agent_locations" + std::to_string(fileID) + ".csv");
}

void UTMDomainAbstract::detectConflicts() {
	agents->detect_conflicts();
	// CURRENTLY CONFLICT DISABLED


	//if (params->_agent_defn_mode==UTMModes::SECTOR){
	//	matrix1d G_c(sectors.size());
	//}

	// count the over capacity here

	// Calculate the amount OVER or UNDER the given capacity
	// COUNT UP SECTOR CAPACITIES

	//matrix2d cap = sectorCapacity;

	// Global reward SECTORS
	/*for (UAV* u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			conflict_count++;

	// D average SECTORS
	cap = sectorCapacity;
	for (UAV* u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				sectors->at(u->curSectorID()).conflicts[j]++; // D avg


	// D downstream SECTORS
	cap = sectorCapacity;
	for (UAV* u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (uint i=0; i<sectors->size(); i++)
				if (u->sectors_touched.find(i)==u->sectors_touched.end())
					conflict_minus_downstream[i]++;	/// D downstream

	// D reallocation SECTORS
	for (uint i=0; i<sectors->size(); i++){
		matrix2d cap_i = cap;
		matrix1d occ_i = cap[i];
		cap_i[i] = matrix1d(cap_i[0].size(),0);

		for (int j=0; j<n_types; j++){
			while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
				// add back up to capacity
				occ_i[j]++;

				int alt;
				do {
					alt = rand()%n_agents;
				} while(alt==i);
				cap_i[alt][j]--;
			}
		}
		for (uint j=0; j<cap_i.size(); j++)
			for (uint k=0; k<cap_i[j].size(); k++)
				if (cap_i[j][k]<0)
					conflict_random_reallocation[i]++;
	}

	// Global reward LINKS
	for (Link* l: links)
		conflict_count += l->get_conflicts();

	// D average LINKS
/*	cap = *linkCapacity;
	for (UAV* u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				linkConflicts[u->curLinkID()][j]++; // D avg
	for (int i=0; i<n_links; i++)
		linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


	// D downstream SECTORS
	cap = *linkCapacity;
	for (UAV* u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			for (int i=0; i<n_links; i++)
				if (u->links_touched.find(i)==u->links_touched.end())
					link_conflict_minus_downstream[i]++;	/// D downstream
					*/
					// D reallocation SECTORS
					/*
					for (int i=0; i<n_links; i++){
						matrix2d cap_i = cap;
						matrix1d occ_i = cap[i];
						cap_i[i] = matrix1d(cap_i[0].size(),0);

						for (int j=0; j<n_types; j++){
							while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
								// add back up to capacity
								occ_i[j]++;

								int alt;
								do {
									alt = rand()%n_links;
								} while(alt==i);
								cap_i[alt][j]--;
							}
						}
						for (uint j=0; j<cap_i.size(); j++)
							for (uint k=0; k<cap_i[j].size(); k++)
								if (cap_i[j][k]<0)
									link_conflict_(om_reallocation[i]++;
					}
					*/
}

void UTMDomainAbstract::getPathPlans() {
	for (UAV* u : UAVs) {
		if (u->t <= 0)	// enforces commitment to link
			u->planAbstractPath();
	}
}

void UTMDomainAbstract::getPathPlans(std::list<UAV* > &new_UAVs) {
	for (UAV* u : new_UAVs) {
		u->planAbstractPath(); // sets own next waypoint
	}
}

void UTMDomainAbstract::reset() {
	//printf("%i UAVs\n",UAVs.size());

	while (!UAVs.empty()) {
		delete UAVs.back();
		UAVs.pop_back();
	}

	for (Link* l : links) {
		l->reset();
	}

	agents->reset();
}

void UTMDomainAbstract::absorbUAVTraffic() {
	// Deletes UAVs
	vector<Link*> l = links;
	UAVs.erase(remove_if(UAVs.begin(), UAVs.end(), [l](UAV* u) {
		if (u->mem == u->mem_end) {
			l [u->cur_link_ID]->remove(u);
			delete u;
			return true;
		}
		else {
			return false;
		}
	}), UAVs.end());
}


void UTMDomainAbstract::getNewUAVTraffic() {
	// Generates (with some probability) plane traffic for each sector
	for (Fix* f : fixes) {
		list<UAV*> new_UAVs = f->generateTraffic(*step);
		for (UAV* u : new_UAVs) {
			UAVs.push_back(u);
			links.at(u->cur_link_ID)->add(u);
		}
	}
}
