#include "ATFMSectorDomain.h"

using namespace std;
using namespace easymath;

ATFMSectorDomain::ATFMSectorDomain(bool deterministic, bool abstraction):
	is_deterministic(deterministic),
	conflict_thresh(10.0),
	delay_sum(0.0)
{
	// Hardcoding of base constants
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_control_elements=n_state_elements*UAV::NTYPES;
	n_steps=100; // steps of simulation time
	n_types=UAV::NTYPES;

	// Object creation
	pathTraces = new vector<vector<XY> >(); // traces the path (reset each epoch)
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();


	// Read in files for sector management
	Load::load_variable(agent_locs, "agent_map/agent_map.csv");
	//Load::load_variable(connection_map,"agent_map/connections.csv");
	Load::load_variable(edges, "agent_map/edges.csv");
	Load::load_variable(&membership_map,"agent_map/membership_map.csv");

	// Add sectors
	for (unsigned int i=0; i<agent_locs.size(); i++){
		sectors->push_back(Sector(agent_locs[i],i, agent_locs.size()));
	}

	n_agents = agent_locs.size(); // number of agents dictated by read in file
	fixed_types=vector<int>(n_agents,0);

	conflict_count = 0; // initialize with no conflicts
	

	if (!abstraction){
		Load::load_variable(fix_locs,"agent_map/fixes.csv");
	
		// Planning
		planners = new AStarManager(UAV::NTYPES, edges, membership_map, agent_locs);
		
		// initialize fixes
		for (unsigned int i=0; i<fix_locs.size(); i++){
			fixes->push_back(Fix(fix_locs[i],i,is_deterministic,planners));
		}
		conflict_count_map = new ID_grid(planners->obstacle_map->dim1(), planners->obstacle_map->dim2());
	}

	return;
}

void ATFMSectorDomain::loadMaps(){

}

ATFMSectorDomain::~ATFMSectorDomain(void)
{
	delete fixes;
	delete sectors;
	delete planners;
	delete conflict_count_map;
	delete pathTraces;
}


vector<double> ATFMSectorDomain::getPerformance(){
	return matrix1d(sectors->size(),-conflict_count);
}

/**
* Loads and capacities are [sector][type]
*/

double ATFMSectorDomain::G(matrix3d &loads, matrix3d &connection_capacities){
	double global_sum=0.0;
	for (int a=0; a<n_agents; a++){
		for (int b=0; b<n_agents; b++){
			for (int t=0; t<UAV::NTYPES; t++){
				double overcap = loads[a][b][t] - connection_capacities[a][b][t];
				if (overcap>0)
					global_sum += overcap; //overcap*overcap;
			}
		}
	}

	return -global_sum;
}

/**
* Go through all the sectors and return loads, format [sector][type]
*/
matrix3d ATFMSectorDomain::getLoads(){
	//[sectorfrom][sectorto][type]

	matrix3d allloads = matrix3d(sectors->size(), matrix2d(sectors->size(), matrix1d(UAV::NTYPES,0.0)));
	for (std::shared_ptr<UAV> &u : UAVs){
		allloads[u->curSectorID()][u->nextSectorID()][u->type_ID]++;
	}
	return allloads;
}

vector<double> ATFMSectorDomain::getRewards(){
	// DELAY REWARD
	return matrix1d(sectors->size(), -delay_sum);
	
	// LINEAR REWARD
	//return matrix1d(sectors->size(),-conflict_count); // linear reward


	// QUADRATIC REWARD
	/*int conflict_sum = 0;
	for (int i=0; i<conflict_count_map->size(); i++){
	for (int j=0; j<conflict_count_map->at(i).size(); j++){
	int c = conflict_count_map->at(i)[j];
	conflict_sum += c*c;
	}
	}
	return matrix1d(sectors->size(),-conflict_sum);*/
}




matrix2d ATFMSectorDomain::getStates(){
	// States: delay assignments for UAVs that need routing
	matrix2d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix1d(n_state_elements,0.0); // reserves space
	}

	for (std::shared_ptr<UAV> &u : UAVs){
		allStates[getSector(u->loc)][u->getDirection()]+=1.0; // Adds the UAV impact on the state
	}

	return allStates;
}

matrix3d ATFMSectorDomain::getTypeStates(){
	matrix3d allStates(n_agents);
	for (int i=0; i<n_agents; i++){
		allStates[i] = matrix2d(n_types);
		for (int j=0; j<n_types; j++){
			allStates[i][j] = matrix1d(n_state_elements,0.0);
		}
	}

	for (std::shared_ptr<UAV> &u: UAVs){
		int a = getSector(u->loc);
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

unsigned int ATFMSectorDomain::getSector(easymath::XY p){
	// tests membership for sector, given a location
	return membership_map->at(p);
}

//HACK: ONLY GET PATH PLANS OF UAVS just generated
void ATFMSectorDomain::getPathPlans(){
	for (std::shared_ptr<UAV> &u : UAVs){
		u->planDetailPath(); // sets own next waypoint
	}
}

void ATFMSectorDomain::getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs){
	for (std::shared_ptr<UAV> &u : new_UAVs){
		u->planDetailPath(); // sets own next waypoint
	}
}

void ATFMSectorDomain::simulateStep(matrix2d agent_actions){
	static int calls=0;
	planners->setCostMaps(agent_actions);
	absorbUAVTraffic();
	//if (calls%10==0) // fakes deterministic?
		getNewUAVTraffic();
	calls++;
	getPathPlans();
	incrementUAVPath();
	detectConflicts();
	//printf("Conflicts %i\n",conflict_count);
}

void ATFMSectorDomain::incrementUAVPath(){
	for (std::shared_ptr<UAV> &u: UAVs)
		u->moveTowardNextWaypoint(); // moves toward next waypoint (next in low-level plan)
	// IMPORTANT! At this point in the code, agent states may have changed
}


void ATFMSectorDomain::absorbUAVTraffic(){
	UAVs.remove_if(at_destination); // NOTE HERE: remove_if may not be removing UAVs!
}


void ATFMSectorDomain::getNewUAVTraffic(){
	//static int calls = 0;
	//static vector<XY> UAV_targets; // making static targets

	// Generates (with some probability) plane traffic for each sector
	list<std::shared_ptr<UAV> > all_new_UAVs;
	for (Fix f: *fixes){
		list<std::shared_ptr<UAV> > new_UAVs = f.generateTraffic(fixes,pathTraces);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);

		// obstacle check
		if (new_UAVs.size() && membership_map->at(new_UAVs.front()->loc)<0){
			printf("issue!");
			exit(1);
		}
	}

	getPathPlans(all_new_UAVs);

	UAVs.splice(UAVs.end(),all_new_UAVs);
	//calls++;
}

void ATFMSectorDomain::reset(){
	UAVs.clear();
	planners->reset();
	conflict_count = 0;
	(*conflict_count_map) = ID_grid(planners->obstacle_map->dim1(), planners->obstacle_map->dim2());
}

void ATFMSectorDomain::logStep(int step){
	// log at 0 and 50
	// no logging, for speed
	/*
	if (step==0){
	pathSnapShot(0);
	}
	if (step==50){
	pathSnapShot(50);
	pathTraceSnapshot();
	//exit(1);
	}*/
}

void ATFMSectorDomain::exportLog(std::string fid, double G){
	static int calls = 0;
	PrintOut::toFileMatrix2D(*conflict_count_map,fid+to_string(calls)+".csv");
	calls++;
}

void ATFMSectorDomain::detectConflicts(){
	for (list<std::shared_ptr<UAV> >::iterator u1=UAVs.begin(); u1!=UAVs.end(); u1++){
		for (list<std::shared_ptr<UAV> >::iterator u2=std::next(u1); u2!=UAVs.end(); u2++){

			double d = easymath::distance((*u1)->loc,(*u2)->loc);

			if (d>conflict_thresh) continue; // No conflict!

 			conflict_count++;

			int midx = ((int)(*u1)->loc.x+(int)(*u2)->loc.x)/2;
			int midy = ((int)(*u1)->loc.y+(int)(*u2)->loc.y)/2;
			conflict_count_map->at(midx,midy)++;

			if ((*u1)->type_ID==UAV::FAST || (*u2)->type_ID==UAV::FAST){
				conflict_count+=10; // big penalty for high priority ('fast' here)
			}


			// DELAY ADDITION
			int delay_add = 5; // add 5 seconds of delay
			if ((*u1)->t_delay>0 || (*u2)->t_delay>0){
				// one is stopped, continue
				continue;
			} else if (rand()%2==0){
				// Select the first one to stop
				(*u1)->t_delay = delay_add;
				delay_sum+=delay_add;
			} else {
				(*u2)->t_delay = delay_add;
				delay_sum+= delay_add;
			}
		}
	}

}

// PATH SNAPSHOT OUTPUT
void ATFMSectorDomain::pathSnapShot(int snapnum){
	matrix2d pathsnaps = matrix2d(2*UAVs.size());
	int ind = 0; // index of path given
	for (std::shared_ptr<UAV> &u:UAVs){
		// pop through queue
		std::queue<XY> wpt_save = u->target_waypoints;
		while (u->target_waypoints.size()){
			pathsnaps[ind].push_back(u->target_waypoints.front().x);
			pathsnaps[ind+1].push_back(u->target_waypoints.front().y);
			u->target_waypoints.pop();
		}
		ind+=2;
		u->target_waypoints = wpt_save;
	}
	PrintOut::toFile2D(pathsnaps,"path-" + to_string(snapnum)+".csv");
}

void ATFMSectorDomain::pathTraceSnapshot(){
	// Exports all path traces
	matrix2d pathsnaps =matrix2d(2*pathTraces->size());
	for (unsigned int i=0, ind=0; i<pathTraces->size(); i++, ind+=2){
		for (unsigned int j=0; j<pathTraces->at(i).size(); j++){
			pathsnaps[ind].push_back(pathTraces->at(i)[j].x);
			pathsnaps[ind+1].push_back(pathTraces->at(i)[j].y);
		}
	}

	PrintOut::toFile2D(pathsnaps,"trace.csv");
}