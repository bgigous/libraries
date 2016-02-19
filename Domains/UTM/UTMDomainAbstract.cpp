#include "UTMDomainAbstract.h"

using namespace std;
using namespace easymath;

UTMDomainAbstract::UTMDomainAbstract(UTMModes* params):
	IDomainStateful(params),
	filehandler(new UTMFileNames(params)),
	params(params)
{
	// Parameters construction
	if (params==NULL) params = new UTMModes(); // use all defaults
	
	// Airspace construction
	int n_sectors;
	switch (params->_airspace_mode){
	case UTMModes::SAVED:
		//highGraph = new TypeGraphManager("agent_map/edges.csv","agent_map/agent_map.csv",n_types);
		highGraph = new TypeGraphManager("edges.csv","nodes.csv",n_types);
		n_sectors = highGraph->getNVertices();
		break;
	case UTMModes::GENERATED:
		n_sectors = params->get_n_sectors();
		highGraph = new TypeGraphManager(n_sectors,n_types,200.0,200.0);
		break;
	default:
		printf("Bad airspace mode. Aborting.");
		system("pause");
		exit(0);
		break;
	}
	params->n_links = highGraph->getEdges().size(); // Must be set after graph created
	n_agents = params->get_n_agents();

	// Link construction
	linkIDs = new map<pair<int,int>,int>();
	vector<vector<int> > connections(n_sectors);
	for (pair<int,int> e: highGraph->getEdges()){
		int source = e.first; // membership of origin of edge
		int target = e.second; // membership of connected node
		XY source_loc = highGraph->getLocation(source);
		XY target_loc = highGraph->getLocation(target);
		int cardinal_dir = cardinal_direction(source_loc - target_loc);

		links.push_back(Link_ptr(new Link(links.size(),source,source_loc,target,target_loc,
			manhattan_distance(source_loc,target_loc)/10.0,
			matrix1d(n_types,double(params->get_flat_capacity())),double(cardinal_dir))));
		linkIDs->insert(make_pair((make_pair(source,target)),links.size()-1));

		connections[source].push_back(target);
	}

	// Sector construction
	for (int i=0; i<n_sectors; i++)
		sectors.push_back(Sector_ptr(new Sector(highGraph->getLocation(i),i, n_sectors, connections[i],params)));

	
	if (params->_agent_defn_mode==UTMModes::SECTOR) agents = new SectorAgentManager(links,n_types,sectors,params);
	else agents = new LinkAgentManager(links.size(),n_types,links,params);
	
	// Fix construction
	for (Sector_ptr &s: sectors)
		fixes.push_back(Fix_ptr(new Fix(s->xy,s->ID,highGraph, NULL, &fixes,params,linkIDs)));
}

string UTMDomainAbstract::createExperimentDirectory(){
	return filehandler->createExperimentDirectory();
}

UTMDomainAbstract::~UTMDomainAbstract(void)
{
}

double UTMDomainAbstract::getGlobalReward(){
	return -agents->global();
	// NEED TO REDO REWARD STUFF
/*
if (params->_agent_defn_mode==UTMModes::SECTOR)
		if (params->_reward_type_mode==UTMModes::RewardType::CONFLICTS)
			return -conflict_count;
		else
			return -delay;
	else if (params->_agent_defn_mode==UTMModes::LINK)
		if (params->_reward_type_mode==UTMModes::RewardType::CONFLICTS){
			double G = 0.0;
			for (Link_ptr &l: links){
				G+=l->conflict_local;
			}
		}
		else 
			return -delay;
	else{
		printf("No _agent_defn_mode set!");
		exit(1000);
	}
	*/
}

matrix1d UTMDomainAbstract::getPerformance(){
	return matrix1d(n_agents,getGlobalReward());
}


matrix1d UTMDomainAbstract::getDifferenceReward(){
	matrix1d G = matrix1d(n_agents,getGlobalReward());
	matrix1d G_c = zeros(n_agents);
		
	if (params->_reward_mode==UTMModes::DIFFERENCE_AVG){
		for (int i=0; i<n_agents; i++){
			G_c[i] = -sum(agents->metrics[i].G_avg);
		}
	} else if (params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM){
		for (int i=0; i<n_agents; i++){
			G_c[i] = -sum(agents->metrics[i].G_minus_downstream);
		}
	}
	
	return G-G_c;

	// REMOVE THE AGENT FROM THE SYSTEM
	/*
	for (unsigned int i=0; i<sectors->size(); i++){
		for (int j=0; j<n_types; j++){
			conflict_node_average[i] += sectors->at(i).conflicts[j];
		}
		conflict_node_average[i] /= double(sectors->at(i).steps);
	}

	for (int i=0; i<n_links; i++){
		for (int j=0; j<n_types; j++){
			link_conflict_node_average[i] += linkConflicts[i][j];
		}
		link_conflict_node_average[i] /= double(linkSteps[i]);
	}


	matrix1d D(n_agents,0.0);
	double G_reg = -getGlobalReward();

	// METHOD 1: INFINITE LINK COSTS (*resim) // NOT FUNCTIONAL YET
	// METHOD 2: STATIC LINK COSTS (*resim) // NOT FUNCTIONAL YET
	// METHOD 3: HAND-CODED LINK COSTS (*resim) // NOT FUNCTIONAL YET

	for (int i=0; i<n_agents; i++){
		switch(params->_reward_mode){
		case UTMModes::DIFFERENCE_DOWNSTREAM:	// METHOD 4: DOWNSTREAM EFFECTS REMOVED
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(G_reg - conflict_minus_downstream[i]);
			else D[i] = -(G_reg - link_conflict_minus_downstream[i]);
			break;
		case UTMModes::DIFFERENCE_DOWNSTREAM_SQ:	// Method 4.1 squareed w/downstream removed
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(pow(G_reg,2.0) - pow(conflict_minus_downstream[i],2.0));
			else D[i] = -(pow(G_reg,2.0) - pow(link_conflict_minus_downstream[i],2.0));
			break;
		case UTMModes::DIFFERENCE_TOUCHED:		// METHOD 5: UPSTREAM AND DOWNSTREAM EFFECTS REMOVED
			/*if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(G_reg - conflict_minus_touched[i]); // NOT FUNCTIONAL YET
				*/
	/*		printf("TODO DIFFERENCE TOUCHED");
			break;
		case UTMModes::DIFFERENCE_TOUCHED_SQ:		// METHOD 5.1: UPSTREAM AND DOWNSTREAM EFFECTS REMOVED squared
			/*if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(pow(G_reg,2.0) - pow(conflict_minus_touched[i],2.0)); // NOT FUNCTIONAL YET
			*/
		/*	printf("TODO DIFFERENCE TOUCHED");
			break;
		case UTMModes::DIFFERENCE_REALLOC:		// METHOD 6: RANDOM TRAFFIC REALLOCATION
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(G_reg - conflict_random_reallocation[i]);
			else D[i] = -(G_reg - link_conflict_random_reallocation[i]);
			break;
		case UTMModes::DIFFERENCE_REALLOC_SQ:		// METHOD 6.1: RANDOM TRAFFIC REALLOCATION
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(pow(G_reg,2.0) - pow(conflict_random_reallocation[i],2.0));
			else D[i] = -(pow(G_reg,2.0) - pow(link_conflict_random_reallocation[i],2.0));
			break;
		case UTMModes::DIFFERENCE_AVG:			// METHOD 7: CONFLICTS AVERAGED OVER THE NODE'S HISTORY
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -conflict_node_average[i];
			else D[i] = -link_conflict_node_average[i];
			break;
		case UTMModes::DIFFERENCE_AVG_SQ:			// METHOD 7.1: CONFLICTS AVERAGED OVER THE NODE'S HISTORY, squared
			if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(pow(G_reg,2.0)-pow(G_reg - conflict_node_average[i],2.0));

			else D[i] = -(pow(G_reg,2.0)-pow(G_reg - link_conflict_node_average[i],2.0));
			break;
		default:{
			printf("unrecognized mode!");
			system("pause");
			exit(1);
				}
		}
	}*/
	//return D;
	return matrix1d(n_agents,0.0);
}

matrix1d UTMDomainAbstract::getRewards(){
	if (params->_reward_mode==UTMModes::GLOBAL){
		return matrix1d(n_agents,getGlobalReward());
	} else {
		return getDifferenceReward();
	}
}


void UTMDomainAbstract::incrementUAVPath(){
	vector<UAV_ptr> eligible; // Eligible to move to the next link
	copy_if(UAVs.begin(),UAVs.end(),back_inserter(eligible),[](UAV_ptr u){
		if (u->t<=0){
			return true;
		} else {
			u->t--;	
			return false;
		}
	});
	
	if (eligible.empty())
		return;

	if (params->_reward_type_mode==UTMModes::CONFLICTS){
		for (UAV_ptr &u : eligible){
			u->high_path_prev.pop_front();
			u->loc = sectors[u->high_path_prev.front()]->xy;
		}
		return;
	} else {
		try_to_move(eligible); // This moves all UAVs that are eligible and not blocked
		// Only those that cannot move are left in eligible
		for (UAV_ptr &u:eligible){
			agents->add_delay(u);	// adds delay for each eligible UAV not able to move
			
			// counterfactuals
			if (params->_reward_mode==UTMModes::DIFFERENCE_AVG)
				agents->add_average_counterfactual();
			else if (params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM)
				agents->add_downstream_delay_counterfactual(u);
			else continue;
		}
	}
}

void UTMDomainAbstract::try_to_move(vector<UAV_ptr> & eligible_to_move){
	random_shuffle(eligible_to_move.begin(),eligible_to_move.end());
	
	int el_size;
	do {
		vector<UAV_ptr>::iterator it = eligible_to_move.begin();
		el_size = eligible_to_move.size();

		vector<Link_ptr> L = links;
		eligible_to_move.erase(
			remove_if(eligible_to_move.begin(),eligible_to_move.end(),
			[L](UAV_ptr u){
				int n = u->next_link_ID;
				int c = u->cur_link_ID;
				int t = u->type_ID;
				if (!L[n]->at_capacity(t)){
					L[n]->move_from(u,L[c]);
					return true;
				} else {
					return false;
				}
		}
		),eligible_to_move.end());

	} while(el_size != eligible_to_move.size());
}

matrix2d UTMDomainAbstract::getStates(){
	// States: delay assignments for UAVs that need routing
	matrix2d allStates(n_agents,matrix1d(n_state_elements,0.0));

	/* "NORMAL POLARITY" state
	for (UAV_ptr &u : UAVs){
		allStates[getSector(u->loc)][u->getDirection()]+=1.0; // Adds the UAV impact on the state
	}
	*/

	// REVERSED POLARITY STATE
	/*for (UAV_ptr &u : UAVs){
		// COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
		if (u->nextSectorID()==u->curSectorID()) continue;
		else allStates[u->nextSectorID()][u->getDirection()] += 1.0;
	}*/

	// CONGESTION STATE

	if (params->_agent_defn_mode==UTMModes::SECTOR){
		vector<int> sector_congestion_count(n_agents,0);
		for (UAV_ptr &u : UAVs){
			sector_congestion_count[u->curSectorID()]++;
		}
		for (unsigned int i=0; i<sectors.size(); i++){
			for (int conn: sectors[i]->connections){

				allStates[i][cardinal_direction(sectors[i]->xy-sectors[conn]->xy)] += sector_congestion_count[conn];
			}
		}
	} else {
		for (UAV_ptr &u : UAVs)
			allStates[u->curLinkID()][u->type_ID]++;
	}

	return allStates;
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions){
	agents->logAgentActions(agent_actions);
	highGraph->setCostMaps(agents->actions2weights(agent_actions));
	getNewUAVTraffic();
	getPathPlans();
	incrementUAVPath();
	absorbUAVTraffic();
	if (params->_reward_type_mode==UTMModes::CONFLICTS)
		detectConflicts();
}


void UTMDomainAbstract::logStep(){
	logUAVLocations();
}

matrix3d UTMDomainAbstract::getTypeStates(){
	matrix3d allStates = zeros(n_agents,n_types,n_state_elements);

	if (params->_agent_defn_mode==UTMModes::SECTOR){
		for (UAV_ptr &u: UAVs){
			int a = u->curSectorID();
			int id = u->type_ID;
			int dir = u->getDirection();
			if (a<0){
				printf("UAV %i at location %f,%f is in an obstacle.!", u->ID,u->loc.x,u->loc.y);
				system("pause");
			}
			allStates[a][id][dir]+=1.0;
		}
	}
	else {
		// link definition
		for (UAV_ptr &u: UAVs){
			int a = u->cur_link_ID;
			int id = u->type_ID;
			allStates[a][id][0]+=1.0;
		}
	}

	return allStates;
}

void UTMDomainAbstract::logUAVLocations(){
		matrix1d stepLocation;
		for (UAV_ptr u: UAVs){
			stepLocation.push_back(u->loc.x);
			stepLocation.push_back(u->loc.y);
		}
		UAVLocations.push_back(stepLocation);
	}

void UTMDomainAbstract::exportUAVLocations(int fileID){
		FileOut::print_vector(UAVLocations,"visualization/locations"+std::to_string(fileID)+".csv");
	}

void UTMDomainAbstract::exportSectorLocations(int fileID){
		std::vector<easymath::XY> sectorLocations;
		for (Sector_ptr &s: sectors)
			sectorLocations.push_back(s->xy);
		FileOut::print_pair_container(sectorLocations,"visualization/agent_locations"+std::to_string(fileID)+".csv");
	}

void UTMDomainAbstract::detectConflicts(){
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
	/*for (UAV_ptr &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			conflict_count++;

	// D average SECTORS
	cap = sectorCapacity;
	for (UAV_ptr &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				sectors->at(u->curSectorID()).conflicts[j]++; // D avg

				
	// D downstream SECTORS
	cap = sectorCapacity;
	for (UAV_ptr &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (unsigned int i=0; i<sectors->size(); i++)
				if (u->sectors_touched.find(i)==u->sectors_touched.end())
					conflict_minus_downstream[i]++;	/// D downstream

	// D reallocation SECTORS
	for (unsigned int i=0; i<sectors->size(); i++){
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
		for (unsigned int j=0; j<cap_i.size(); j++)
			for (unsigned int k=0; k<cap_i[j].size(); k++)
				if (cap_i[j][k]<0)
					conflict_random_reallocation[i]++;
	}

	// Global reward LINKS
	for (Link_ptr &l: links)
		conflict_count += l->get_conflicts();

	// D average LINKS
/*	cap = *linkCapacity;
	for (UAV_ptr &u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				linkConflicts[u->curLinkID()][j]++; // D avg
	for (int i=0; i<n_links; i++)
		linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


	// D downstream SECTORS
	cap = *linkCapacity;
	for (UAV_ptr &u: UAVs)
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
		for (unsigned int j=0; j<cap_i.size(); j++)
			for (unsigned int k=0; k<cap_i[j].size(); k++)
				if (cap_i[j][k]<0)
					link_conflict_random_reallocation[i]++;
	}
	*/
}

void UTMDomainAbstract::getPathPlans(){
	for (UAV_ptr &u : UAVs){
		u->planAbstractPath();
	}
}

void UTMDomainAbstract::getPathPlans(std::list<UAV_ptr > &new_UAVs){
	for (UAV_ptr &u : new_UAVs){
		u->planAbstractPath(); // sets own next waypoint
	}
}

void UTMDomainAbstract::reset(){
	UAVs.clear();
	UAVLocations.clear();
	for (Link_ptr l:links){
		l->reset();
	}

	agents->agentActions.clear();
	agents->reset();
}

void UTMDomainAbstract::absorbUAVTraffic(){
	// Deletes UAVs
	remove_erase_if(UAVs,[](UAV_ptr u){return u->loc==u->end_loc;});
	for (Link_ptr l:links){
		for (list<UAV_ptr> &t:l->traffic){
			remove_erase_if(t,[](UAV_ptr u){return u->loc==u->end_loc;});
		}
	}
}


void UTMDomainAbstract::getNewUAVTraffic(){
	// Generates (with some probability) plane traffic for each sector
	
	int traffic_sum = 0;
	for (int i=0; i<links.size(); i++){
		for (int t=0; t<links[i]->traffic.size(); t++){
			int tsize = links[i]->traffic[t].size();
			traffic_sum += links[i]->traffic[t].size();
		}
	}
	
	for (Fix_ptr &f: fixes){
		list<UAV_ptr> new_UAVs = f->generateTraffic(*step);
		for (UAV_ptr &u: new_UAVs){
			UAVs.push_back(u);
			links.at(u->cur_link_ID)->add(u);
		}
	}
}