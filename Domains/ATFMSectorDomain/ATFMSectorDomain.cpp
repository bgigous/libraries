#include "ATFMSectorDomain.h"

using namespace std;
using namespace easymath;

ATFMSectorDomain::ATFMSectorDomain(bool deterministic):
	is_deterministic(deterministic), abstraction_mode(true) // hardcode for the abstraction...
{


	pathTraces = new vector<vector<XY> >(); // traces the path (reset each epoch)

	// Object creation
	sectors = new vector<Sector>();
	fixes = new vector<Fix>();
	UAVs = new list<UAV>();

	// inherritance elements: constant
	//n_control_elements=4; // 4 outputs for sectors (cost in cardinal directions) (no types)
	n_control_elements=4*UAV::NTYPES;
	n_state_elements=4; // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	n_steps=100; // steps of simulation time
	n_types=UAV::NTYPES;

	// Read in files for sector management
	obstacle_map = new barrier_grid(256,256);
	membership_map = new ID_grid(256,256); //HARDCODED
	//load_variable(*obstacle_map,"agent_map/obstacle_map.csv",0.0); // last element specifies height threshold for obstacle
	load_variable(*membership_map,"agent_map/membership_map.csv");

	for (int i=0; i<membership_map->dim1(); i++){
		for (int j=0; j<membership_map->dim2(); j++){
			obstacle_map->at(i,j) = membership_map->at(i,j)<0;
		}
	}

	matrix2d agent_coords = FileManip::readDouble("agent_map/agent_map.csv");
	matrix2d connection_map = FileManip::readDouble("agent_map/connections.csv");

	matrix2d fix_locs = FileManip::readDouble("agent_map/fixes.csv");
	if (abstraction_mode) fix_locs = agent_coords; // if using an abstraction, have only the centers phsyically located


	// Add sectors
	agent_locs = vector<XY>(agent_coords.size()); // save this for later Astar recreation
	for (int i=0; i<agent_coords.size(); i++){
		sectors->push_back(Sector(XY(agent_coords[i][0],agent_coords[i][1]),i));
		agent_locs[i] = sectors->back().xy;
	}

	// Adjust the connection map to be the edges
	// preprocess boolean connection map
	// map edges to (sector ind, direction ind)
	for (int i=0; i<connection_map.size(); i++){
		for (int j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j] && i!=j){
				XY xyi = agent_locs[i];
				XY xyj = agent_locs[j];
				XY dx_dy = xyj-xyi;
				int xydir = cardinalDirection(dx_dy);
				int memj = membership_map->at(xyj); // only care about cost INTO sector
				sector_dir_map[edges.size()] = make_pair(memj,xydir); // add at new index
				edges.push_back(AStar_easy::edge(i,j));
			}
		}
	}

	//weights = vector<double>(edges.size(),1.0); //initialization of weights
	weights = matrix2d(UAV::NTYPES);
	for (int i=0; i<weights.size(); i++){
		weights[i] = matrix1d(edges.size(),1.0);
	}

	// initialize fixes
	for (int i=0; i<fix_locs.size(); i++){
		fixes->push_back(Fix(XY(fix_locs[i][0],fix_locs[i][1]),i,is_deterministic));
	}

	n_agents = sectors->size(); // number of agents dictated by read in file
	fixed_types=vector<int>(n_agents,0);

	conflict_count = 0; // initialize with no conflicts

	// Initialize Astar object (must re-create this each time weight change
	Astar_highlevel = std::vector<AStar_easy*>(UAV::NTYPES);
	for (int i=0; i<UAV::NTYPES; i++){
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
	}

	// Add a different A* for each connection
	for (int i=0; i<connection_map.size(); i++){
		for (int j=0; j<connection_map[i].size(); j++){
			if (connection_map[i][j]>0){
				m2astar[i][j] = new AStar_grid(obstacle_map,membership_map,i,j);
			}
		}
	}


	conflict_count_map = new ID_grid(obstacle_map->dim1(), obstacle_map->dim2());


	if (abstraction_mode){
		if (n_agents!=15){
			printf("WRONG CONNECTION SIZE!");
			system("pause");
		}
		for (int i=0; i<n_agents; i++){
			for (int j=0; j<n_agents; j++){
				XY diff = agent_locs[i]-agent_locs[j];
				connection_time[i][j] = abs(diff.x)+abs(diff.y);
				for (int k=0; k<UAV::NTYPES; k++){
					connection_capacity[i][j][k] = 10; // flat capacity across all types on the connection
				}
			}
		}
	}

	edge_time = vector<vector<int> >(n_agents);
	for (int i=0; i<n_agents; i++) edge_time[i] = vector<int>(n_agents,10); // time cost to be on an edge
}

ATFMSectorDomain::~ATFMSectorDomain(void)
{
	delete UAVs;
	delete fixes;
	delete sectors;
	delete obstacle_map;
	delete membership_map;

	// Delete all pointers: recreate later...
	for (grid_lookup::iterator it=m2astar.begin(); it!=m2astar.end(); it++){
		for (std::map<int,AStar_grid*>::iterator inner=it->second.begin(); inner!=it->second.end(); inner++){
			delete inner->second;
		}
	}

	delete conflict_count_map;
	clear_all(Astar_highlevel);
	delete pathTraces;
}


vector<double> ATFMSectorDomain::getPerformance(){
	if (abstraction_mode){
		return getRewards();
	} else {
		return matrix1d(sectors->size(),-conflict_count);
	}
}

/**
* Loads and capacities are [sector][type]
*/
double ATFMSectorDomain::G(vector<vector<int> > loads, vector<vector<int> > capacities){
	double global=0;
	for (int i=0; i<loads.size(); i++){
		for (int j=0; j<loads[i].size(); j++){
			double overcap = loads[i][j] - capacities[i][j];
			global -= overcap*overcap; // worse with higher concentration of planes!
		}
	}
	return global;
}

/**
* Go through all the sectors and return loads, format [sector][type]
*/
vector<vector<int> > ATFMSectorDomain::getLoads(){
	vector<vector<int> > allloads = vector<vector<int> >(sectors->size());
	for (int i=0; i<sectors->size(); i++){
		allloads[i] = sectors->at(i).getLoad();
	}
	return allloads;
}

vector<double> ATFMSectorDomain::getRewards(){
	// LINEAR REWARD
	if (!abstraction_mode){
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
	} else {
		// Calculate loads
		vector<Demographics> oldLoads = getLoads(); // get the current loads on all sectors
		vector<vector<Demographics> > allloads = vector<vector<Demographics> >(n_agents); // agent removed, agent for load, type
		for (int i=0; i< allloads.size(); i++){
			allloads[i] = oldLoads;
			allloads[i][i] = Demographics(UAV::NTYPES,0); // traffic removed, added back in later
		}


		// Count the adjusted load
		for(Sector s: *sectors){

			// Go through each UAV in the sector
			// Get the 'toward' load, currently
			//loads[s.sectorID] = oldLoads[s.sectorID]; // taken care of above


			// build the map with the blacked-out sector
			matrix2d mod_weights = weights;
			for (int i=0; i<mod_weights.size(); i++){
				mod_weights[i][s.sectorID] = 9999999.99;
			}

			resetGraphWeights(mod_weights);

			for (UAV* u: s.toward){
				// Get the 'reroute' load, later
				// plan a path using a generic A* with modified weights
				int memstart = membership_map->at(u->loc.x,u->loc.y);
				int memend = membership_map->at(u->end_loc.x,u->end_loc.y);
				list<AStar_easy::vertex> plantemp = Astar_highlevel[u->type_ID]->search(memstart,memend);

				int newnextsector = plantemp.front();
				allloads[s.sectorID][newnextsector][u->type_ID]++;
			}
			resetGraphWeights(weights); // reset the cost maps

		}

		// Calculate D from counterfactual
		vector<Demographics> C = vector<Demographics>(n_agents);// capacities[agent, type]
		for (int i=0; i<C.size(); i++){
			C[i] = Demographics(UAV::NTYPES,10);
		}
		matrix1d D = matrix1d(n_agents);
		for (int i=0; i<n_agents; i++){
			double G_reg = G(oldLoads,C);
			double G_c = G(allloads[i],C);
			D[i] = G_reg-G_c;
		}
		// 

		return D; // global reward
	}
}


void ATFMSectorDomain::printMasks(){
	for (grid_lookup::iterator it=m2astar.begin(); it!=m2astar.end(); it++){
		for (map<int,AStar_grid*>::iterator inner = it->second.begin(); inner!=it->second.end(); inner++){
			inner->second->m.printMap("masks/", it->first, inner->first);
		}
	}
}

void ATFMSectorDomain::load_variable(std::vector<std::vector<bool> >* var, std::string filename, double thresh, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);
	*var = std::vector<std::vector<bool> >(f.size());

	for (int i=0; i<f.size(); i++){
		var->at(i) = std::vector<bool>(f[i].size());
		for (int j=0; j<f[i].size(); j++){
			if (atof(f[i][j].c_str())<=thresh){
				var->at(i)[j] = false;
			} else {
				var->at(i)[j] = true;
			}
		}
	}
}


void ATFMSectorDomain::load_variable(Matrix<bool,2> &var, std::string filename, double thresh, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);

	for (int i=0; i<f.size(); i++){
		for (int j=0; j<f[i].size(); j++){
			if (atof(f[i][j].c_str())<=thresh){
				var(i,j) = false;
			} else {
				var(i,j) = true;
			}
		}
	}
}

void ATFMSectorDomain::load_variable(Matrix<int,2> &var, std::string filename, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);

	for (int i=0; i<f.size(); i++){
		for (int j=0; j<f[i].size(); j++){
			var(i,j) = atoi(f[i][j].c_str());
		}
	}
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
	vector<XY> sectorLocations = vector<XY>(sectors->size());
	for (int i=0; i<sectors->size(); i++){
		sectorLocations[i] = sectors->at(i).xy;
	}

	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		u->pathPlan(Astar_highlevel[u->type_ID],m2astar,obstacle_map,membership_map,sectorLocations, abstraction_mode, connection_time); // sets own next waypoint
	}
}

void ATFMSectorDomain::getPathPlans(std::list<UAV> &new_UAVs){
	
	vector<XY> sectorLocations = vector<XY>(sectors->size());
	for (int i=0; i<sectors->size(); i++){
		sectorLocations[i] = sectors->at(i).xy;
	}

	for (list<UAV>::iterator u=new_UAVs.begin(); u!=new_UAVs.end(); u++){
		u->pathPlan(Astar_highlevel[u->type_ID],m2astar,obstacle_map,membership_map,sectorLocations, abstraction_mode, connection_time); // sets own next waypoint
	}
}

void ATFMSectorDomain::simulateStep(matrix2d agent_actions){
	static int calls=0;
	setCostMaps(agent_actions);
	absorbUAVTraffic();
	if (calls%10==0)
		getNewUAVTraffic();
	calls++;
	getPathPlans();
	incrementUAVPath();
	detectConflicts();
	//printf("Conflicts %i\n",conflict_count);
}

void ATFMSectorDomain::resetGraphWeights(matrix2d weightset){
	weights = weightset;

	for (int i=0; i<Astar_highlevel.size(); i++){
		delete Astar_highlevel[i];
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]); // replace existing weights
	}
}

void ATFMSectorDomain::setCostMaps(vector<vector<double> > agent_actions){
	// [next sector][direction of travel] -- current
	// agent_actions  = agent, [type, dir<-- alternating]

	for (int i=0; i<weights[0].size(); i++){
		for (int j=0; j<UAV::NTYPES; j++){
			int s = sector_dir_map[i].first;
			//int d = j*UAV::NTYPES + sector_dir_map[i].second; // wrong

			int d = j + sector_dir_map[i].second*UAV::NTYPES;
			weights[j][i] = agent_actions[s][d];
		}
		// HACK
		// weights[i] = agent_actions[sector_dir_map[i].first][sector_dir_map[i].second]; // AGENT SETUP // old
		// weights[i] = 1.0; // NO AGENTS
	}

	for (int i=0; i<Astar_highlevel.size(); i++){
		delete Astar_highlevel[i];
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]); // replace existing weights
	}
}

void ATFMSectorDomain::incrementUAVPath(){
	if (!abstraction_mode){
		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			u->moveTowardNextWaypoint(); // moves toward next waypoint (next in low-level plan)
		}
	} else {
		// in abstraction mode, move to next center of target
		for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
			if (u->time_left_on_edge <=0){
				u->loc = sectors->at(*u->high_path_prev.begin()).xy;
				u->high_path_prev.pop_front();
			} else {
				u->time_left_on_edge--;
			}
		}
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
	// moved here for efficiency
	list<UAV> cur_UAVs;
	for (list<UAV>::iterator u=UAVs->begin(); u!=UAVs->end(); u++){
		if (u->loc==u->end_loc){
			// don't copy over
		} else {

			// invalid plans no longer created
			/*if (u->target_waypoints.size() && u->target_waypoints.front()==loc){ // deleted if size==0; drop invalid plans
			u->target_waypoints.pop();
			}*/
			cur_UAVs.push_back(*u);
		}
	}
	(*UAVs) = cur_UAVs; // copy over


	/*for (int i=0; i<fixes->size(); i++){
	fixes->at(i).absorbTraffic(UAVs);
	}*/
}


void ATFMSectorDomain::getNewUAVTraffic(){
	//static int calls = 0;
	//static vector<XY> UAV_targets; // making static targets

	// Generates (with some probability) plane traffic for each sector
	list<UAV> all_new_UAVs;
	for (int i=0; i<fixes->size(); i++){
		list<UAV> new_UAVs = fixes->at(i).generateTraffic(fixes, obstacle_map,pathTraces);
		all_new_UAVs.splice(all_new_UAVs.end(),new_UAVs);

		// obstacle check
		if (new_UAVs.size() && membership_map->at(new_UAVs.front().loc)<0){
			printf("issue!");
			exit(1);
		}
	}

	getPathPlans(all_new_UAVs);

	UAVs->splice(UAVs->end(),all_new_UAVs);
	//calls++;
}

void ATFMSectorDomain::reset(){
	static int calls=0;

	// Drop all UAVs
	//printf("UAVs = %i\n",UAVs->size());
	UAVs->clear();
	// reset weights
	conflict_count = 0;
	//
	//weights = vector<double>(edges.size(),1.0); //initialization of weights
	weights = matrix2d(UAV::NTYPES);
	for (int i=0; i<weights.size(); i++){
		weights[i] = matrix1d(edges.size(), 1.0);
	}

	// re-create high level a*

	for (int i=0; i<Astar_highlevel.size(); i++){
		delete Astar_highlevel[i];
		Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
	}

	// re-create low level a*
	/*for (map<list<AStar_easy::vertex>, AStar_easy* >::iterator it=astar_lowlevel.begin(); it!=astar_lowlevel.end(); it++){
	delete it->second;
	}
	astar_lowlevel.clear();*/

	//PrintOut::toFile(*conflict_count_map,"conflict_map.csv");
	// clear conflict count map
	for (int i=0; i<conflict_count_map->dim1(); i++){
		for (int j=0; j<conflict_count_map->dim2(); j++){
			conflict_count_map->at(i,j) = 0; // set all 0
		}
	}

	// reset the path trace so it doesn't get too big
	// pathTraces->clear();
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

	if (abstraction_mode){
		// CONFLICT COUNTING DONE IN REWARD... based on overcap
	} else {
		double conflict_thresh = 1.0;
		for (list<UAV>::iterator u1=UAVs->begin(); u1!=UAVs->end(); u1++){
			for (list<UAV>::iterator u2=std::next(u1); u2!=UAVs->end(); u2++){
				double d = easymath::distance(u1->loc,u2->loc);

				if (u1!=u2){
					if (d<conflict_thresh){
						conflict_count++;

						int avgx = (u1->loc.x+u2->loc.x)/2;
						int avgy = (u1->loc.y+u2->loc.y)/2;
						conflict_count_map->at(avgx,avgy)++;
						if (u1->type_ID==UAV::FAST || u2->type_ID==UAV::FAST){
							conflict_count+=10; // big penalty for high priority ('fast' here)
						}
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