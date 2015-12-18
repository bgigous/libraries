#include "UTMDomainAbstract.h"


UTMDomainAbstract::UTMDomainAbstract():
	// MODES
	_capacity_mode(TWO),
	_nagents_mode(FIFTEEN),
	_reward_mode(GLOBAL),
	_airspace_mode(GENERATED)
{

	// Hardcoding of base constants
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_control_elements=n_state_elements*UAV::NTYPES;
	n_steps=100; // steps of simulation time// TESTING PARAMETER: CHANGED FROM 100
	n_types=UAV::NTYPES;


	switch (_nagents_mode){
	case FIVE:
		n_agents = 5;
		break;
	case TEN:
		n_agents = 10;
		break;
	case FIFTEEN:
		n_agents = 15;
		break;
	case TWENTY:
		n_agents = 20;
		break;
	case THIRTY:
		n_agents = 30;
		break;
	case FORTY:
		n_agents = 40;
		break;
	case FIFTY:
		n_agents = 50;
		break;
	default:
		printf("unknown nagents mode");
	}

	switch(_capacity_mode){
	case TWO:
		flat_capacity = 2;
		break;
	case FOUR: 
		flat_capacity = 4;
		break;
	case SIX:
		flat_capacity = 6;
		break;
	case EIGHT:
		flat_capacity = 8;
		break;
	default:
		printf("unknown capacity mode");
	}

	// Object creation
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	
	if (_airspace_mode == SAVED){
		//airspace = new AirspaceMap("agent_map/agent_map.csv","agent_map/edges.csv", n_types,2.0);
		//n_agents = airspace->agentLocs.size();
		highGraph = new TypeGraphManager("agent_map/edges.csv","agent_map/agent_map.csv",n_types);
		n_agents = highGraph->getNAgents();
	} else if(_airspace_mode == GENERATED){
		//airspace = new AirspaceMap(n_agents, n_types, 2.0);
		highGraph = new TypeGraphManager(n_agents,n_types,200.0,200.0);
	}

	// Add sectors
	for (int i=0; i<n_agents; i++){
		sectors->push_back(Sector(highGraph->getLocation(i),i, n_agents, vector<int>()));
		sectors->at(i).conflicts = matrix1d(n_types,0.0);
	}
	for (pair<int,int> i: highGraph->getEdges()){
		sectors->at(i.first).connections.push_back(i.second);
	}
	sectorCapacity = matrix2d(n_agents,matrix1d(n_types, flat_capacity));
	connectionTime = matrix2d(n_agents,matrix1d(n_agents,0.0));
	for (unsigned int i=0; i<sectors->size(); i++){
		for (unsigned int j=0; j<sectors->size(); j++){
			connectionTime[i][j] = manhattanDist(sectors->at(i).xy,sectors->at(j).xy);
		}
	}

	fixed_types=vector<int>(n_agents,0);

	conflict_count = 0; // initialize with no conflicts
	conflict_minus_downstream = matrix1d(n_agents,0.0);
	conflict_minus_touched = matrix1d(n_agents, 0.0);
	conflict_node_average = matrix1d(n_agents,0.0);
	conflict_random_reallocation = matrix1d(n_agents,0.0);
	
	// initialize fixes
	for (unsigned int i=0; i<sectors->size(); i++){
		fixes->push_back(Fix(sectors->at(i).xy,i,highGraph, NULL, fixes));
	}
}


UTMDomainAbstract::~UTMDomainAbstract(void)
{
}

double UTMDomainAbstract::getGlobalReward(){
	return -conflict_count;
}

matrix1d UTMDomainAbstract::getPerformance(){
	return matrix1d(n_agents,getGlobalReward());
}

matrix1d UTMDomainAbstract::getDifferenceReward(){
	// REMOVE THE AGENT FROM THE SYSTEM

	for (unsigned int i=0; i<sectors->size(); i++){
		for (int j=0; j<UAV::NTYPES; j++){
			conflict_node_average[i] += sectors->at(i).conflicts[j];
		}
		conflict_node_average[i] /= double(sectors->at(i).steps);
	}

	
	matrix1d D(n_agents,0.0);
	double G_reg = conflict_count;
	
	// METHOD 1: INFINITE LINK COSTS (*resim) // NOT FUNCTIONAL YET
	// METHOD 2: STATIC LINK COSTS (*resim) // NOT FUNCTIONAL YET
	// METHOD 3: HAND-CODED LINK COSTS (*resim) // NOT FUNCTIONAL YET
	// METHOD 4: DOWNSTREAM EFFECTS REMOVED
	for (int i=0; i<n_agents; i++){
		if (_reward_mode==DIFFERENCE_DOWNSTREAM)
			D[i] = -(G_reg - conflict_minus_downstream[i]);

		else if (_reward_mode==DIFFERENCE_TOUCHED)
		// METHOD 5: UPSTREAM AND DOWNSTREAM EFFECTS REMOVED
			D[i] = -(G_reg - conflict_minus_touched[i]); // NOT FUNCTIONAL YET

		else if (_reward_mode==DIFFERENCE_REALLOC)
		// METHOD 6: RANDOM TRAFFIC REALLOCATION
			D[i] = -(G_reg - conflict_random_reallocation[i]);
	
		else if (_reward_mode==DIFFERENCE_AVG)
		// METHOD 7: CONFLICTS AVERAGED OVER THE NODE'S HISTORY
		D[i] = -conflict_node_average[i];// -(G_reg - conflict_node_average[i]); // NOT FUNCTIONAL YET
		
		else{
			printf("unrecognized mode!");
			system("pause");
			exit(1);
		}
	}
	return D;

}

matrix1d UTMDomainAbstract::getLocalReward(){
	matrix1d L(n_agents,0.0);
	for (int i=0; i<n_agents; i++){
		for (int j=0; j<UAV::NTYPES; j++){
			L[i] += sectors->at(i).conflicts[j];
		}
	}
	return L;
}

matrix1d UTMDomainAbstract::getRewards(){
	// Calculate loads
	if (_reward_mode==GLOBAL){
		return matrix1d(n_agents, getGlobalReward());
	} else if (_reward_mode==DIFFERENCE_AVG ||
		_reward_mode == DIFFERENCE_DOWNSTREAM ||
		_reward_mode == DIFFERENCE_REALLOC ||
		_reward_mode == DIFFERENCE_TOUCHED){
		return getDifferenceReward();
	} else {
		printf("No reward type set.");
		system("pause");
		exit(1);
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
	vector<int> sector_congestion_count(n_agents,0);
	for (std::shared_ptr<UAV> &u : UAVs){
		sector_congestion_count[u->curSectorID()]++;
	}
	for (int i=0; i<n_agents; i++){
		for (int conn: sectors->at(i).connections){

			allStates[i][cardinalDirection(sectors->at(i).xy-sectors->at(conn).xy)] += sector_congestion_count[conn];
		}
	}

	return allStates;
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions, int step){
	highGraph->setCostMaps(agent_actions);
	absorbUAVTraffic();
	getNewUAVTraffic(step);
	getPathPlans();
	incrementUAVPath();
	detectConflicts();
}


void UTMDomainAbstract::logStep(int step){
}

matrix3d UTMDomainAbstract::getTypeStates(){
	matrix3d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix2d(n_types);
		for (int j=0; j<n_types; j++){
			allStates[i][j] = matrix1d(n_state_elements,0.0);
		}
	}

	for (std::shared_ptr<UAV> &u: UAVs){
		int a = u->curSectorID();
		int id = u->type_ID;
		int dir = u->getDirection();
		if (a<0){
			printf("UAV %i at location %f,%f is in an obstacle.!", u->ID,u->loc.x,u->loc.y);
			system("pause");
		}
		allStates[a][id][dir]+=1.0;
	}

	return allStates;
}

void UTMDomainAbstract::detectConflicts(){
	// count the over capacity here

	// Calculate the amount OVER or UNDER the given capacity
	matrix2d cap = sectorCapacity;

	for (std::shared_ptr<UAV> &u: UAVs){
		double c = cap[u->curSectorID()][u->type_ID]--;
		if (c<0){
			conflict_count++;
			for  (int j=0; j<UAV::NTYPES; j++){
				sectors->at(u->curSectorID()).conflicts[j]++;
			}
			for (unsigned int i=0; i<sectors->size(); i++){
				if (u->sectors_touched.find(i)==u->sectors_touched.end()){
					// Not touched, still counted
					conflict_minus_downstream[i]++;
				}
			}
		}
	}

	for (int i=0; i<n_agents; i++){
		sectors->at(i).steps++; // steps of conflict accounting (for average counterfactual)
		// random reallocation

		matrix2d cap_i = cap;
		matrix1d occ_i = cap[i];
		cap_i[i] = matrix1d(cap_i[0].size(),0);
		
		for (int j=0; j<UAV::NTYPES; j++){
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
		for (unsigned int j=0; j<cap_i.size(); j++){
			for (unsigned int k=0; k<cap_i[j].size(); k++){
				if (cap_i[j][k]<0){
					conflict_random_reallocation[i]++;
				}
			}
		}
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
		sectors->at(i).conflicts = matrix1d(UAV::NTYPES,0.0);
	}

	UAVs.clear();
	conflict_count = 0; // initialize with no conflicts
	conflict_minus_downstream = matrix1d(n_agents,0.0);
	conflict_minus_touched = matrix1d(n_agents, 0.0);
	conflict_node_average = matrix1d(n_agents,0.0);
	conflict_random_reallocation = matrix1d(n_agents,0.0);
}

void UTMDomainAbstract::absorbUAVTraffic(){
	UAVs.remove_if(at_destination);
}


void UTMDomainAbstract::getNewUAVTraffic(int step){

	// Generates (with some probability) plane traffic for each sector
	list<std::shared_ptr<UAV> > all_new_UAVs;
	for (Fix f: *fixes){
		list<std::shared_ptr<UAV> > new_UAVs = f.generateTraffic(step);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);
	}

	UAVs.splice(UAVs.end(),all_new_UAVs);
}

string UTMDomainAbstract::createExperimentDirectory(){
	// Creates a directory for the experiment and then returns that as a string
	// DIRECTORY HIERARCHY: EXPERIMENTS/NAGENTS/TRAFFIC/CAPACITY/REWARDTYPE/
	// typehandling(file name).csv assumed
	string AGENT_FOLDER = EXPERIMENT_FOLDER+to_string(n_agents)+"_Agents/";
	string TRAFFIC_FOLDER;
	if (Fix::_traffic_mode==Fix::DETERMINISTIC){
		TRAFFIC_FOLDER = AGENT_FOLDER + "Deterministic_" + to_string(fixes->at(0).gen_rate) + "_Traffic/";
	} else {
		TRAFFIC_FOLDER = AGENT_FOLDER + "Probabilistic_" + to_string(fixes->at(0).p_gen*100) + "_Traffic/";
	}
	string CAPACITY_FOLDER = TRAFFIC_FOLDER + to_string(sectorCapacity[0][0]) + "_Capacity/"; // assume uniform sector capacity
	string REWARD_FOLDER = CAPACITY_FOLDER + to_string(_reward_mode) + "_Reward/";

	_mkdir(EXPERIMENT_FOLDER);
	_mkdir(AGENT_FOLDER.c_str());
	_mkdir(TRAFFIC_FOLDER.c_str());
	_mkdir(CAPACITY_FOLDER.c_str());
	_mkdir(REWARD_FOLDER.c_str());


	return REWARD_FOLDER;
}

string UTMDomainAbstract::getRewardModeName(RewardMode mode){
	string reward_names[RewardMode::NMODES] = {
		"GLOBAL-", 
		"DIFFERENCE_DOWNSTREAM-",
		"DIFFERENCE_TOUCHED-",
		"DIFFERENCE_REALLOC-",
		"DIFFERENCE_AVG-",
	};

	return reward_names[mode];
}