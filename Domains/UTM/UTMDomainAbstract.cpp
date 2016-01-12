#include "UTMDomainAbstract.h"


UTMDomainAbstract::UTMDomainAbstract(UTMModes* params, UTMFileNames* setfilehandler):
	filehandler(setfilehandler), params(params)
{
	if (params==NULL) params = new UTMModes(); // use all defaults
	if (setfilehandler==NULL) filehandler = new UTMFileNames(params);
	
	// Base constants
	n_state_elements	=	params->get_n_state_elements();
	n_control_elements	=	params->get_n_control_elements();
	n_steps				=	params->get_n_steps();
	n_types				=	params->get_n_types();
	int n_sectors		=	params->get_n_sectors();
	
	// AIRSPACE GENERATION
	switch (params->_airspace_mode){
	case UTMModes::SAVED:
		{
			highGraph = new TypeGraphManager("agent_map/edges.csv","agent_map/agent_map.csv",n_types);
			n_sectors = highGraph->getNVertices();
			break;
		}
	case UTMModes::GENERATED:
		highGraph = new TypeGraphManager(n_sectors,n_types,200.0,200.0);
		break;
	}

	if (params->_agent_defn_mode==UTMModes::SECTOR){
		n_agents = params->get_n_sectors();
		printf("# sector agents = %i",n_agents);
	}
	else if (params->_agent_defn_mode==UTMModes::LINK){
		n_agents = highGraph->getEdges().size();
		printf("# link agents = %i",n_agents);
	}

	n_links = highGraph->getEdges().size();
	
	// Object creation/basic initialization
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	fixed_types=vector<int>(n_agents,0);
	conflict_count = 0; // initialize with no conflicts
	conflict_minus_downstream = matrix1d(n_sectors,0.0);	/// COUNT OVER EACH AGENT
	conflict_minus_touched = matrix1d(n_sectors, 0.0);
	conflict_node_average = matrix1d(n_sectors,0.0);
	conflict_random_reallocation = matrix1d(n_sectors,0.0);
	link_conflict_count = 0; // initialize with no conflicts
	link_conflict_minus_downstream = matrix1d(n_links,0.0);	/// COUNT OVER EACH AGENT
	link_conflict_minus_touched = matrix1d(n_links, 0.0);
	link_conflict_node_average = matrix1d(n_links,0.0);
	link_conflict_random_reallocation = matrix1d(n_links,0.0);


	sectorCapacity = matrix2d(n_agents,matrix1d(n_types, params->get_flat_capacity()));
	linkCapacity = matrix2d(n_agents,matrix1d(n_types, params->get_flat_capacity()));
	connectionTime = matrix2d(n_sectors,matrix1d(n_sectors,0.0));

	linkConflicts = matrix2d(n_links,matrix1d(n_types,0.0));
	linkSteps = matrix1d(n_links,0.0);
	
	// SECTORS
	for (int i=0; i<n_sectors; i++){
		sectors->push_back(Sector(highGraph->getLocation(i),i, n_sectors, vector<int>(),params));
		sectors->at(i).conflicts = matrix1d(n_types,0.0);
	}

	// Directions and connections
	
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)

	int edgeNum = 0;
	for (pair<int,int> e: highGraph->getEdges()){
		int memi = e.first; // membership of origin of edge
		int memj = e.second; // membership of connected node
		XY xyi = sectors->at(memi).xy;
		XY xyj = sectors->at(memj).xy;
		XY dx_dy = xyj-xyi;
		int xydir = cardinalDirection(dx_dy);
		sector_dir_map[edgeNum++] = make_pair(memj,xydir); // add at new index
		sectors->at(memi).connections.push_back(memj);
	}
	
	// TODO: CREATE LINK MANAGER HERE
	if (params->_agent_defn_mode==UTMModes::SECTOR) agents = new SectorAgentManager(sector_dir_map);
	else agents = new LinkAgentManager();

	for (unsigned int i=0; i<sectors->size(); i++){
		for (unsigned int j=0; j<sectors->size(); j++){
			connectionTime[i][j] = manhattanDist(sectors->at(i).xy,sectors->at(j).xy);
		}
	}
	
	// FIXES
	for (unsigned int i=0; i<sectors->size(); i++){
		fixes->push_back(Fix(sectors->at(i).xy,i,highGraph, NULL, fixes,params));
	}

	
}

string UTMDomainAbstract::createExperimentDirectory(){
	return filehandler->createExperimentDirectory();
}

UTMDomainAbstract::~UTMDomainAbstract(void)
{
}

double UTMDomainAbstract::getGlobalReward(){
	if (params->_agent_defn_mode==UTMModes::SECTOR)
		return -conflict_count;
	else
		return -link_conflict_count;
}

double UTMDomainAbstract::getGlobalRewardSquared(){
	return -getGlobalReward()*getGlobalReward();
}

matrix1d UTMDomainAbstract::getPerformance(){
	if (params->_reward_mode==UTMModes::GLOBAL_SQ || 
		params->_reward_mode==UTMModes::DIFFERENCE_AVG_SQ || 
		params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM_SQ || 
		params->_reward_mode==UTMModes::DIFFERENCE_TOUCHED_SQ)
		return matrix1d(n_agents,-pow(getGlobalReward(),2.0));
	else
		return matrix1d(n_agents,getGlobalReward());
}

matrix1d UTMDomainAbstract::getDifferenceReward(){
	// REMOVE THE AGENT FROM THE SYSTEM
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
			printf("TODO DIFFERENCE TOUCHED");
			break;
		case UTMModes::DIFFERENCE_TOUCHED_SQ:		// METHOD 5.1: UPSTREAM AND DOWNSTREAM EFFECTS REMOVED squared
			/*if (params->_agent_defn_mode==UTMModes::SECTOR)
				D[i] = -(pow(G_reg,2.0) - pow(conflict_minus_touched[i],2.0)); // NOT FUNCTIONAL YET
			*/
			printf("TODO DIFFERENCE TOUCHED");
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
	}
	return D;
}

matrix1d UTMDomainAbstract::getLocalReward(){
	matrix1d L(n_agents,0.0);
	for (int i=0; i<n_agents; i++){
		for (int j=0; j<n_types; j++){
			L[i] += sectors->at(i).conflicts[j];
		}
	}
	return L;
}

matrix1d UTMDomainAbstract::getRewards(){
	// Calculate loads
	switch (params->_reward_mode){
	case UTMModes::GLOBAL:
		return matrix1d(n_agents, getGlobalReward());
	case UTMModes::GLOBAL_SQ:
		return matrix1d(n_agents, -getGlobalReward()*getGlobalReward());
	default:
		return getDifferenceReward();
	}
}

void UTMDomainAbstract::incrementUAVPath(){
	// in abstraction mode, move to next center of target
	for (std::shared_ptr<UAV> &u: UAVs){
		if (u->pathChanged){
			u->t = int(connectionTime[u->curSectorID()][u->nextSectorID()]);
		} else if (u->t <=0){
			u->high_path_prev.pop_front();
			u->loc = sectors->at(u->high_path_prev.front()).xy;
		} else {
			u->t--;
		}
	}
}


matrix2d UTMDomainAbstract::getStates(){
	// States: delay assignments for UAVs that need routing
	matrix2d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix1d(n_state_elements,0.0); // reserves space
	}

	/* "NORMAL POLARITY" state
	for (std::shared_ptr<UAV> &u : UAVs){
		allStates[getSector(u->loc)][u->getDirection()]+=1.0; // Adds the UAV impact on the state
	}
	*/

	// REVERSED POLARITY STATE
	/*for (std::shared_ptr<UAV> &u : UAVs){
		// COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
		if (u->nextSectorID()==u->curSectorID()) continue;
		else allStates[u->nextSectorID()][u->getDirection()] += 1.0;
	}*/

	// CONGESTION STATE

	if (params->_agent_defn_mode){
		vector<int> sector_congestion_count(n_agents,0);
		for (std::shared_ptr<UAV> &u : UAVs){
			sector_congestion_count[u->curSectorID()]++;
		}
		for (int i=0; i<sectors->size(); i++){
			for (int conn: sectors->at(i).connections){

				allStates[i][cardinalDirection(sectors->at(i).xy-sectors->at(conn).xy)] += sector_congestion_count[conn];
			}
		}
	} else {
		for (std::shared_ptr<UAV> &u : UAVs)
			allStates[u->curLinkID()][u->type_ID]++;
	}

	return allStates;
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions, int step){
	agents->logAgentActions(agent_actions);
	highGraph->setCostMaps(agents->actions2weights(agent_actions,n_types,highGraph->getEdges().size()));
	absorbUAVTraffic();
	getNewUAVTraffic(step);
	getPathPlans();
	incrementUAVPath();
	detectConflicts();
}


void UTMDomainAbstract::logStep(int step){
	logUAVLocations();
}

matrix3d UTMDomainAbstract::getTypeStates(){
	matrix3d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix2d(n_types);
		for (int j=0; j<n_types; j++){
			allStates[i][j] = matrix1d(n_state_elements,0.0);
		}
	}

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
			int a = u->curLinkID();
			int id = u->type_ID;
			allStates[a][id][0]+=1.0;
		}
	}

	return allStates;
}

void UTMDomainAbstract::detectConflicts(){
	// count the over capacity here

	// Calculate the amount OVER or UNDER the given capacity
	// COUNT UP SECTOR CAPACITIES

	matrix2d cap = sectorCapacity;

	// Global reward SECTORS
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			conflict_count++;

	// D average SECTORS
	cap = sectorCapacity;
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				sectors->at(u->curSectorID()).conflicts[j]++; // D avg
	for (int i=0; i<sectors->size(); i++)
		sectors->at(i).steps++; // steps of conflict accounting (for average counterfactual)


	// D downstream SECTORS
	cap = sectorCapacity;
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curSectorID()][u->type_ID]--)<0)
			for (unsigned int i=0; i<sectors->size(); i++)
				if (u->sectors_touched.find(i)==u->sectors_touched.end())
					conflict_minus_downstream[i]++;	/// D downstream

	// D reallocation SECTORS
	for (int i=0; i<sectors->size(); i++){
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
	cap = linkCapacity;
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			link_conflict_count++;

	// D average LINKS
	cap = linkCapacity;
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			for (int j=0; j<n_types; j++)
				linkConflicts[u->curLinkID()][j]++; // D avg
	for (int i=0; i<n_links; i++)
		linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


	// D downstream SECTORS
	cap = linkCapacity;
	for (std::shared_ptr<UAV> &u: UAVs)
		if ((cap[u->curLinkID()][u->type_ID]--)<0)
			for (unsigned int i=0; i<n_links; i++)
				if (u->links_touched.find(i)==u->links_touched.end())
					link_conflict_minus_downstream[i]++;	/// D downstream

	// D reallocation SECTORS
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


}

void UTMDomainAbstract::getPathPlans(){
	
	for (std::shared_ptr<UAV> &u : UAVs){
		u->planAbstractPath();
	}
}

void UTMDomainAbstract::getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs){

	for (std::shared_ptr<UAV> &u : new_UAVs){
		u->planAbstractPath(); // sets own next waypoint
	}
}

void UTMDomainAbstract::reset(){

	for (unsigned int i=0; i<sectors->size(); i++){
		sectors->at(i).steps=0;
		sectors->at(i).conflicts = matrix1d(n_types,0.0);
	}

	UAVs.clear();

	static int calls = 0; // temporary incrementing system
	//exportUAVLocations(calls);
	//exportAgentLocations(calls);
	//exportAgentActions(calls);
	calls++;

	UAVLocations.clear();
	agents->agentActions.clear();
	conflict_count = 0; // initialize with no conflicts
	conflict_minus_downstream = matrix1d(n_agents,0.0);
	conflict_minus_touched = matrix1d(n_agents, 0.0);
	conflict_node_average = matrix1d(n_agents,0.0);
	conflict_random_reallocation = matrix1d(n_agents,0.0);

	
	linkConflicts = matrix2d(n_links,matrix1d(n_types,0.0));
	linkSteps = matrix1d(n_links,0.0);
	link_conflict_count=0;
	link_conflict_minus_downstream=matrix1d(n_links,0.0);
	link_conflict_minus_touched=matrix1d(n_links,0.0);
	link_conflict_node_average=matrix1d(n_links,0.0);
	link_conflict_random_reallocation=matrix1d(n_links,0.0);
}

void UTMDomainAbstract::absorbUAVTraffic(){
	UAVs.remove_if(at_destination);
}


void UTMDomainAbstract::getNewUAVTraffic(int step){

	// Generates (with some probability) plane traffic for each sector
	list<UAV_ptr> all_new_UAVs;
	for (Fix f: *fixes){
		list<UAV_ptr> new_UAVs = f.generateTraffic(step);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);
	}

	UAVs.splice(UAVs.end(),all_new_UAVs);
}
