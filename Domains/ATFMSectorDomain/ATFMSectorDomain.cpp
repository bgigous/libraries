#include "ATFMSectorDomain.h"

using namespace std;
using namespace easymath;

UAV::UAV(XY start_loc, XY end_loc,std::vector<std::vector<XY> > *pathTraces, UAVType t):
	loc(start_loc), end_loc(end_loc), ID(pathTraces->size()), pathTraces(pathTraces), type_ID(t)
{
	pathTraces->push_back(vector<XY>(1,loc)); // new path trace created for each UAV,starting at location

	switch(t){
	case SLOW:
		speed = 1;
		break;
	case FAST:
		// substitute: high priority
		speed = 1;
		//speed = 2;
		break;
	default:{
		printf("no speed found!!");
		system("pause");
		break;
			}
	}

};

void UAV::pathPlan(AStar_easy* Astar_highlevel, vector<vector<bool> >*obstacle_map,
				   vector<vector<int> >* membership_map, vector<Sector>* sectors, 
				   map<list<AStar_easy::vertex>, AStar_easy* > &astar_lowlevel)
{

	int memstart = membership_map->at(loc.x)[loc.y];
	int memend =  membership_map->at(end_loc.x)[end_loc.y];
	list<AStar_easy::vertex> high_path = Astar_highlevel->search(memstart,memend);

	if(high_path_prev !=high_path){
		high_path_prev = high_path;
		AStar_easy* astar_low;
		if (astar_lowlevel.count(high_path)){
			astar_low = astar_lowlevel[high_path];
		} else{
			// MAKE A NEW LOW-LEVEL ASTAR
			astar_low = new AStar_easy(high_path,obstacle_map,membership_map);
			astar_lowlevel[high_path] = astar_low;
		}
		list<AStar_easy::vertex> low_path= astar_low->search(loc,end_loc);
		while (target_waypoints.size()) target_waypoints.pop();
		for (list<AStar_easy::vertex>::iterator it=low_path.begin(); it!=low_path.end(); it++){
			target_waypoints.push(astar_low->rlookup[*it]); // adds to list to visit
		}
		target_waypoints.pop(); // removes CURRENT location from target
	}
}

int UAV::getDirection(){
	// Identifies whether traveling in one of four cardinal directions
	return cardinalDirection(loc-end_loc);
}

Sector::Sector(XY xy): xy(xy)
{
}

Fix::Fix(XY loc, bool deterministic): is_deterministic(deterministic), loc(loc), p_gen(0.05) // change to 1.0 if traffic controlled elsewhere
{
}

bool Fix::atDestinationFix(const UAV &u){
	double dist_thresh = 2.0;
	return u.target_waypoints.size()					// UAV has planned a trajectory
		&& u.target_waypoints.front()==loc				// UAV wants to go there next
		&& easymath::distance(u.loc,loc)<dist_thresh	// UAV is close enough
		&& u.end_loc==loc;								// This is destination fix
}

std::list<UAV> Fix::generateTraffic(vector<Fix>* allFixes, vector<vector<bool> >* obstacle_map,std::vector<std::vector<XY> > *pathTraces){
	// Creates a new UAV in the world
	static int calls = 0;

	std::list<UAV> newTraffic;


	/*if (is_deterministic){
		if (calls%10==0){
			XY end_loc = allFixes->at(calls%allFixes->size()).loc;
			if (end_loc==loc) end_loc = allFixes->at(calls%(allFixes->size()+1)).loc;

			// HACK
			UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES));
			if (type_id_set==UAV::FAST) end_loc = allFixes->at(0).loc;
			else end_loc = allFixes->at(0).loc;
			// HACK

			newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set)); //  even UAV types
		}
	} else {*/
		double coin = COIN_FLOOR0;
		if (coin<p_gen){
			XY end_loc = allFixes->at(COIN_FLOOR0*allFixes->size()).loc;
			while (end_loc==loc){
				end_loc = allFixes->at(COIN_FLOOR0*allFixes->size()).loc;
			}

			// HACK
			UAV::UAVType type_id_set = UAV::UAVType(calls%int(UAV::UAVType::NTYPES));
			/*
			if (type_id_set==UAV::FAST) end_loc = allFixes->at(0).loc;
			else end_loc = allFixes->at(0).loc;
			*/
			// HACK

			newTraffic.push_back(UAV(loc,end_loc,pathTraces,type_id_set)); //  even UAV types
		//}
	}
	calls++;
	return newTraffic;
}

void Fix::absorbTraffic(list<UAV>* UAVs){
	double dist_thresh = 2.0;
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




	//list<UAV> cur_UAVs;
	/*UAVs->remove_if(atDestinationFix);
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
	u->target_waypoints.pop();
	}*/


	/*for(list<UAV>::reverse_iterator u=UAVs->rbegin(); u!=UAVs->rend(); u++){
	if (atDestinationFix(*u)){
	UAVs->erase(std::next(u).base());
	//printf("Erased!\n");
	} else {
	if (u->target_waypoints.size() && u->target_waypoints.front()==loc) 
	u->target_waypoints.pop();
	}
	}*/
}

ATFMSectorDomain::ATFMSectorDomain(bool deterministic):
	is_deterministic(deterministic)
{
	pathTraces = new vector<vector<XY> >(); // traces the path (reset each epoch)

	// Object creation
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	UAVs = new list<UAV>();

	// inherritance elements: constant
	n_control_elements=4; // 4 outputs for sectors (cost in cardinal directions)
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_steps=100; // steps of simulation time
	n_types=UAV::NTYPES;

	// Read in files for sector management
	obstacle_map = new vector<vector<bool> >();
	membership_map = new vector<vector<int> >();
	load_variable(obstacle_map,"agent_map/obstacle_map.csv",0.0); // last element specifies height threshold for obstacle
	DataManip::load_variable(membership_map,"agent_map/membership_map.csv");

	/*
	matrix2d obstacle_map_double = FileManip::readDouble("agent_map/obstacle_map.csv");
	matrix2d membership_map_double = FileManip::readDouble("agent_map/membership_map.csv");


	obstacle_map = new vector<vector<bool> >(obstacle_map_double.size());
	for (int i=0; i<obstacle_map_double.size(); i++){
		obstacle_map->at(i) = vector<bool>(obstacle_map_double[i].size(),true);
		for (int j=0; j<obstacle_map_double[i].size(); j++){
			obstacle_map->at(i)[j] = (obstacle_map_double[i][j]>0); // returns true if obstacle, false otherwise
		}
	}

	membership_map = new vector<vector<int> >(membership_map_double.size());
	for (int i=0; i<membership_map_double.size(); i++){
		membership_map->at(i) = vector<int>(membership_map_double[i].size(),0);
		for (int j=0; j<membership_map_double[i].size(); j++){
			membership_map->at(i)[j] = int(membership_map_double[i][j]);
		}
	}
	*/

	matrix2d agent_coords = FileManip::readDouble("agent_map/agent_map.csv");
	matrix2d connection_map = FileManip::readDouble("agent_map/connections.csv");
	matrix2d fix_locs = FileManip::readDouble("agent_map/fixes.csv");

	// Add sectors
	agent_locs = vector<XY>(agent_coords.size()); // save this for later Astar recreation
	for (int i=0; i<agent_coords.size(); i++){
		sectors->push_back(Sector(XY(agent_coords[i][0],agent_coords[i][1])));
		agent_locs[i] = XY(agent_coords[i][0],agent_coords[i][1]);
	}

	// Adjust the connection map to be the edges
	// preprocess boolean connection map
	// map edges to (sector ind, direction ind)
	edges; // save this for later Astar recreation
	for (int i=0; i<connection_map.size(); i++){
		for (int j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j] && i!=j){
				XY xyi = agent_locs[i];
				XY xyj = agent_locs[j];
				XY dx_dy = xyj-xyi;
				int xydir = cardinalDirection(dx_dy);
				int memj = membership_map->at(xyj.x)[xyj.y]; // only care about cost INTO sector
				sector_dir_map[edges.size()] = make_pair(memj,xydir); // add at new index
				edges.push_back(AStar_easy::edge(i,j));
			}
		}
	}
	weights = vector<double>(edges.size(),1.0); //initialization of weights

	// initialize fixes
	for (int i=0; i<fix_locs.size(); i++){
		fixes->push_back(Fix(XY(fix_locs[i][0],fix_locs[i][1]),is_deterministic));
	}

	n_agents = sectors->size(); // number of agents dictated by read in file
	fixed_types=vector<int>(n_agents,0);

	conflict_count = 0; // initialize with no conflicts

	// Initialize Astar object (must re-create this each time weight change
	AStar_easy Astar_highlevel = AStar_easy(agent_locs,edges,weights);


	conflict_count_map = new vector<vector<int> >(obstacle_map->size());
	for (int i=0; i<conflict_count_map->size(); i++){
		conflict_count_map->at(i) = vector<int>(obstacle_map->at(i).size(),0);
	}
}

ATFMSectorDomain::~ATFMSectorDomain(void)
{
	delete UAVs;
	delete fixes;
	delete sectors;
	delete obstacle_map;
	delete membership_map;
	for (map<list<AStar_easy::vertex>, AStar_easy* >::iterator it=astar_lowlevel.begin();
		it!=astar_lowlevel.end(); it++){
			delete it->second;
	}
	delete conflict_count_map;
	delete Astar_highlevel;
	delete pathTraces;
}


vector<double> ATFMSectorDomain::getPerformance(){
	return matrix1d(sectors->size(),-conflict_count);
}

vector<double> ATFMSectorDomain::getRewards(){
	// LINEAR REWARD
	return matrix1d(sectors->size(),-conflict_count); // linear reward


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

	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
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

	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		int a = getSector(u->loc);
		int id = u->type_ID;
		int dir = u->getDirection();
		allStates[a][id][dir]+=1.0;
	}

	return allStates;
}

unsigned int ATFMSectorDomain::getSector(easymath::XY p){
	// tests membership for sector, given a location
	return membership_map->at(p.x)[p.y];
}

void ATFMSectorDomain::getPathPlans(){
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		u->pathPlan(Astar_highlevel,obstacle_map,membership_map,sectors,astar_lowlevel); // sets own next waypoint
	}
}

void ATFMSectorDomain::simulateStep(matrix2d agent_actions){
	//static int calls = 0;
	setCostMaps(agent_actions);
	absorbUAVTraffic();
	//if (calls%50==0) getNewUAVTraffic(); // steady traffic
	getNewUAVTraffic(); // probabilistic traffic
	//calls++;
	getPathPlans();
	incrementUAVPath();
	detectConflicts();
	//printf("Conflicts %i\n",conflict_count);
}

void ATFMSectorDomain::setCostMaps(vector<vector<double> > agent_actions){
	for (int i=0; i<weights.size(); i++){
		weights[i] = agent_actions[sector_dir_map[i].first][sector_dir_map[i].second]; // AGENT SETUP
		//weights[i] = 1.0; // NO AGENTS
	}

	delete Astar_highlevel;
	Astar_highlevel = new AStar_easy(agent_locs,edges,weights); // replace existing weights
}

void ATFMSectorDomain::incrementUAVPath(){
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		u->moveTowardNextWaypoint(); // moves toward next waypoint (next in low-level plan)
	}
	// IMPORTANT! At this point in the code, agent states may have changed
}

void UAV::moveTowardNextWaypoint(){
	for (int i=0; i<speed; i++){
		if (!target_waypoints.size()) return; // return if no waypoints
		loc = target_waypoints.front();
		pathTraces->at(ID).push_back(loc);
		target_waypoints.pop();
	}
}

void ATFMSectorDomain::absorbUAVTraffic(){
	for (int i=0; i<fixes->size(); i++){
		fixes->at(i).absorbTraffic(UAVs);
	}
}


void ATFMSectorDomain::getNewUAVTraffic(){
	//static vector<XY> UAV_targets; // making static targets

	// Generates (with some probability) plane traffic for each sector
	list<UAV> all_new_UAVs;
	for (int i=0; i<fixes->size(); i++){
		list<UAV> new_UAVs = fixes->at(i).generateTraffic(fixes, obstacle_map,pathTraces);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);
	}


	//MAKE STATIC
	/*if (UAV_targets.size()){
	int count=0;
	for (list<UAV>::iterator i=all_new_UAVs.begin(); i!=all_new_UAVs.end(); i++){
	i->end_loc = UAV_targets[count];
	count++;
	}
	} else {
	for (list<UAV>::iterator i=all_new_UAVs.begin(); i!=all_new_UAVs.end(); i++){
	UAV_targets.push_back(i->end_loc);
	}
	}*/
	// END MAKE STATIC

	UAVs->splice(UAVs->end(),all_new_UAVs);
}

void ATFMSectorDomain::reset(){
	static int calls=0;

	// Drop all UAVs
	//printf("UAVs = %i\n",UAVs->size());
	UAVs->clear();
	// reset weights
	conflict_count = 0;
	//
	weights = vector<double>(edges.size(),1.0); //initialization of weights
	// re-create high level a*
	delete Astar_highlevel;
	Astar_highlevel = new AStar_easy(agent_locs,edges,weights);

	// re-create low level a*
	for (map<list<AStar_easy::vertex>, AStar_easy* >::iterator it=astar_lowlevel.begin(); it!=astar_lowlevel.end(); it++){
		delete it->second;
	}
	astar_lowlevel.clear();

	//  EXPORT IF THIS IS THE BEST - moved to epoch()
	/*static int calls0 = 0; // calls to output 0
	static int calls99 = 0;
	// EXPORT CONFLICT MAP FIRST!
	if (calls==0){
	PrintOut::
	(*conflict_count_map,"stat_results/conflict_map-0-"+to_string(calls0)+".csv");
	calls0++;
	}
	if (calls==(100*20-1)){ // 100 epochs, 20 nn -- only halfway through!
	PrintOut::toFile(*conflict_count_map,"stat_results/conflict_map-99-"+to_string(calls99)+".csv");
	calls99++;
	calls = 0; // reset calls
	}
	calls++;
	*/


	//PrintOut::toFile(*conflict_count_map,"conflict_map.csv");
	// clear conflict count map
	for (int i=0; i<conflict_count_map->size(); i++){
		for (int j=0; j<conflict_count_map->at(i).size(); j++){
			conflict_count_map->at(i)[j] = 0; // set all 0
		}
	}


	// reset the path trace so it doesn't get too big
	// pathTraces->clear();
}

void ATFMSectorDomain::logStep(int step){
	// log at 0 and 50
	if (step==0){
		pathSnapShot(0);
	}
	if (step==50){
 		pathSnapShot(50);
		pathTraceSnapshot();
		//exit(1);
	}
}

void ATFMSectorDomain::exportLog(std::string fid, double G){
	static int calls = 0;
	PrintOut::toFile2D(*conflict_count_map,fid+to_string(calls)+".csv");
	calls++;
}

void ATFMSectorDomain::detectConflicts(){
	double conflict_thresh = 1.0;
	for (list<UAV>::iterator u1=UAVs->begin(); u1!=UAVs->end(); u1++){
		for (list<UAV>::iterator u2=std::next(u1); u2!=UAVs->end(); u2++){
			double d = easymath::distance(u1->loc,u2->loc);
			
/* size type
			double large_thresh = 5*conflict_thresh;

			if (u1!=u2){
				if (((u1->type_ID+u2->type_ID)==0 && d<conflict_thresh)
					|| ((u1->type_ID+u2->type_ID)==1 && d<(conflict_thresh+large_thresh)/2)
					|| (((u1->type_ID+u2->type_ID)==2 && d<large_thresh))){
						conflict_count++;

						int avgx = (u1->loc.x+u2->loc.x)/2;
						int avgy = (u1->loc.y+u2->loc.y)/2;
						conflict_count_map->at(avgx)[avgy]++;
				}
			}
			*/
			if (u1!=u2){
				if (d<conflict_thresh){
						conflict_count++;

						int avgx = (u1->loc.x+u2->loc.x)/2;
						int avgy = (u1->loc.y+u2->loc.y)/2;
						conflict_count_map->at(avgx)[avgy]++;
						if (u1->type_ID==UAV::FAST || u2->type_ID==UAV::FAST){
							conflict_count+=10; // big penalty for high priority ('fast' here)
						}
				}
			}

		}
	}
}

// PATH SNAPSHOT OUTPUT
void ATFMSectorDomain::pathSnapShot(int snapnum){
	matrix2d pathsnaps = matrix2d(2*UAVs->size());
	int ind = 0; // index of path given
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
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
	for (int i=0, ind=0; i<pathTraces->size(); i++, ind+=2){
		for (int j=0; j<pathTraces->at(i).size(); j++){
			pathsnaps[ind].push_back(pathTraces->at(i)[j].x);
			pathsnaps[ind+1].push_back(pathTraces->at(i)[j].y);
		}
	}

	PrintOut::toFile2D(pathsnaps,"trace.csv");
}