#include "RoverDomain.h"

using namespace std;
using namespace easymath;


RoverDomainParameters::RoverDomainParameters():
	IDomainStatefulParameters(),
	rover_types_fixed(vector<int>(get_n_agents(), Rover::SLOWTURN))
{
	// distance bound
	distance_bounds.clear();
	distance_bounds.push_back(0.0);
	distance_bounds.push_back(5.0);	// close bound
	distance_bounds.push_back(10.0);// medium bound
	// Anything past 10 is far

}

RoverDomainParameters::~RoverDomainParameters() {

}

POI::POI(XY p) : XY(p), val(10.0) {
}

POI::~POI() {

}

int RoverDomain::distance_discretization(XY &p1, XY &p2) {
	double dist = euclidean_distance(p1, p2);
	return bin(dist, domain_params->distance_bounds);
}

RoverDomain::RoverDomain(void) :
	domain_params(new RoverDomainParameters()),
	IDomainStateful(domain_params),
	deltaO(10.0),
	world(new GridWorld()),
	rovers(vector<Rover*>(domain_params->get_n_agents())),
	POIs(domain_params->get_n_POI())
{
	fixedRoverPositions = world->get_unique_positions(n_agents);
	for (int i = 0; i < n_agents; i++) {
		rovers[i] = new Rover(world, domain_params);
		rovers[i]->set_x(fixedRoverPositions[i].x);
		rovers[i]->set_y(fixedRoverPositions[i].y);
	}

	fixedPOIPositions = world->get_unique_positions(domain_params->get_n_POI());
	for (size_t i = 0; i < POIs.size(); i++) {
		POIs[i] = new POI(fixedPOIPositions[i]);
	}
}


RoverDomain::~RoverDomain(void)
{
	delete world;
	delete domain_params;

	for (size_t i = 0; i < rovers.size(); i++) {
		delete rovers[i];
	}

	for (size_t i = 0; i < POIs.size(); i++) {
		delete POIs[i];
	}
}

void RoverDomain::reset() {
	//resets the position of the rovers
	for (size_t i = 0; i < fixedRoverPositions.size(); i++) {
		(XY)*rovers[i] = fixedRoverPositions[i];
	}
}

void RoverDomain::simulateStep(matrix2d agent_actions) {
	for (size_t i = 0; i < rovers.size(); i++) {
		// Adjust actions to match type
		if (rovers[i]->type == Rover::ERRATIC) {
			if (easymath::rand(0, 1) < 0.2) {
				// Take a random action
				agent_actions[i][0] = easymath::rand(0, 0.99);
				agent_actions[i][1] = easymath::rand(0, 0.99);
			}
		}
		else if (rovers[i]->type == Rover::FAST) {
			agent_actions[i][0] *= 10.0;
			agent_actions[i][1] *= 10.0;
		}
		else if (rovers[i]->type == Rover::SLOWTURN) {
			agent_actions[i][0] /= 10.0;
			agent_actions[i][0] /= 10.0;
		}

		(easymath::XY)*rovers[i] = (easymath::XY)*rovers[i] + easymath::XY(agent_actions[i][0], agent_actions[i][1]);
	}
}

matrix1d RoverDomain::getRewards() {
	double G_sum = 0.0;
	for (size_t i = 0; i<POIs.size(); i++) {
		matrix1d d = matrix1d(rovers.size());
		for (size_t j = 0; j < rovers.size(); j++)
		{
			d[j] = easymath::euclidean_distance(*POIs[i], *rovers[j]);
			if (d[j] < 1.0) d[j] = 1.0;	 // bound closeness
		}
		std::sort(d.begin(), d.end());

		double Nij = d[0] < domain_params->get_observation_distance();
		double Nik = d[1] < domain_params->get_observation_distance();
		G_sum += 2.0*(Nij*Nik) / (d[0] + d[1]);
	}
	return matrix1d(rovers.size(), G_sum);
}

vector<Rover*> RoverDomain::get_neighbors(Rover* ref, vector<Rover*> all_rovers, int n_neighbors) {
	sort(all_rovers.begin(), all_rovers.end(), [ref](Rover* r1, Rover* r2) {
		if (r1->ID == ref->ID) return false;
		else if (r2->ID == ref->ID) return true;
		else return (euclidean_distance(*r1, *ref) < euclidean_distance(*r2, *ref));
	});

	return vector<Rover*>(rovers.begin(), rovers.begin() + n_neighbors);
}

matrix1d RoverDomain::rover_quadrant_count(Rover* ref) {
	// Count number of rovers in each quadrant as percentage of total rovers
	matrix1d directional_count = zeros(4);
	for (Rover* r: rovers) {
		if (r->ID == ref->ID)
			continue;
		else {
			int Qj = cardinal_direction(*ref - *r); // Quadrant of rover j, relative to i
			directional_count[Qj] += 1.0 / double(rovers.size() - 1);
		}
	}

	return directional_count;
}

matrix1d RoverDomain::POI_quadrant_count(Rover* ref) {
	matrix1d directional_count = zeros(4);
	for (POI* p:POIs) {
		int Qj = cardinal_direction(*ref - *p); // Quadrant of POI j, relative to i
		directional_count[Qj] += 1.0 / double(POIs.size()); // Adding this to the state (offset by earlier state elements)
	}
	return directional_count;
}

matrix3d RoverDomain::getTypeStates() {
	matrix3d S = zeros(n_agents, n_types, n_state_elements);

	for (size_t i = 0; i<rovers.size(); i++) {
		vector<Rover*> neighbors = get_neighbors(rovers[i],rovers,1);

		int t = neighbors[0]->type;

		matrix1d s = rover_quadrant_count(rovers[i]);
		matrix1d s_pois = POI_quadrant_count(rovers[i]);
		s.insert(s.end(), s_pois.begin(), s_pois.end());

		S[i][t] = s;
	}
	return S;
}

matrix2d RoverDomain::getStates() {
	matrix2d S = zeros(n_agents, n_state_elements);

	for (size_t i = 0; i<rovers.size(); i++) {
		matrix1d s = rover_quadrant_count(rovers[i]);
		matrix1d s_pois = POI_quadrant_count(rovers[i]);
		s.insert(s.end(), s_pois.begin(), s_pois.end());

		S[i] = s;
	}

	return S;
}

matrix1d RoverDomain::getPerformance() {
	return matrix1d(n_agents, getGlobalReward());
}

std::vector<double> RoverDomain::getDifferenceReward() {
	//If more than two robots visit a POI, only the observations of the closest two are considered,
	// and their visit distances are averaged in the computation of the system evaluation

	std::vector<double> D(rovers.size(), 0.0); // D for all of the rovers
	typedef std::pair<double, int> P;

	for (size_t i = 0; i<POIs.size(); i++) {
		POI* p = POIs[i];

		// Get all distances to POI
		vector<pair<double,int> > d = vector<pair<double,int> > (rovers.size());
		for (size_t j = 0; j < rovers.size(); j++)
		{
			d[j].first = easymath::euclidean_distance(*POIs[i], *rovers[j]);
			if (d[j].first < 1.0) d[j].first = 1.0;	 // bound closeness
			d[j].second = j;
		}
		std::sort(d.begin(), d.end());


		// Get top 3 closest (dij, dik, dil) in order closest to farthest
		double dij = d[0].first;
		size_t j = d[0].second;
		double dik = d[1].first;
		size_t k = d[1].second;
		double dil = d[2].first;
		size_t l = d[2].second;

		// BEGIN MODIFICATION
		double multiplej = rovers[j]->get_type_multiplier();
		double multiplek = rovers[k]->get_type_multiplier();
		double multiplel = rovers[l]->get_type_multiplier();

		double gatheredValjk = p->val*(multiplej + multiplek);
		double gatheredValjl = p->val*(multiplej + multiplel);
		double gatheredValkl = p->val*(multiplek + multiplel);
		// END MODIFICATION


		if (dil<deltaO) {
			//D[unsigned(j)]+=2.0*p.val*(1.0/(dik+dij+2.0)-1.0/(dik+dil+2.0)); // original
			//D[unsigned(k)]+=2.0*p.val*(1.0/(dik+dij+2.0)-1.0/(dij+dil+2.0)); // original
			D[j] += gatheredValjk / (dik + dij + 2.0) - gatheredValkl / (dik + dil + 2.0); // modified
		}
		else if ((dij<deltaO) && (dik<deltaO)) {
			D[j] += 2.0*gatheredValjk / (dik + dij + 2.0); // reduces to original
			D[k] += 2.0*gatheredValjk / (dik + dij + 2.0);
		}
	}
	return D;
}


double RoverDomain::getLocalReward(int me) {
	// Returns Pj(z)
	double L = 0.0;
	for (size_t i = 0; i<POIs.size(); i++) {
		double deltaMe = euclidean_distance(*rovers[me], *POIs[i]);
		if (deltaMe<deltaO) {
			double gatheredVal = POIs[i]->val / deltaMe;

			// BEGIN VALUE MODIFICATION*********
			if (rovers[me]->type == 0) {
				gatheredVal *= 0.25;
			}
			else if (rovers[me]->type == 2) {
				gatheredVal *= 0.5;
			}
			else if (rovers[me]->type == 3) {
				gatheredVal *= 0.75;
			}
			// END VALUE MODIFICATION************

			L += gatheredVal;
		}
	}

	return L;
}

double RoverDomain::getGlobalReward() {
	//If more than two robots visit a POI, only the observations of the closest two are considered,
	// and their visit distances are averaged in the computation of the system evaluation

	double G = 0.0;
	for (size_t i = 0; i<POIs.size(); i++) {
		POI *p = POIs[i];

		// Get all distances to POI
		vector<pair<double,int> > d = vector<pair<double, int> >(rovers.size());
		for (size_t j = 0; j < rovers.size(); j++)
		{
			d[j].first = easymath::euclidean_distance(*POIs[i], *rovers[j]);
			if (d[j].first < 1.0)
				d[j].first = 1.0;	 // bound closeness
		}
		std::sort(d.begin(), d.end());


		double dij = d[0].first; // closest, DISTANCE
		int jID = d[0].second; // closest, ID
		double dik = d[i].first; // second closest, DISTANCE
		int kID = d[i].second; // second closest, ID

		double Nij = 0.0;
		double Nik = 0.0;
		if (dij<deltaO) {
			Nij = 1.0;
		}
		if (dik<deltaO) {
			Nik = 1.0;
		}

		// BEGIN MODIFICATION
		double gatheredVal; // modification
		double multiplej, multiplek; // modification
									 // average the multiples based on type...
		if (rovers[jID]->type == 0) {
			multiplej = 0.25;
		}
		else if (rovers[jID]->type == 2) {
			multiplej = 0.5;
		}
		else if (rovers[jID]->type == 3) {
			multiplej = 0.75;
		}
		else {
			multiplej = 1.0;
		}
		if (rovers[kID]->type == 0) {
			multiplek = 0.25;
		}
		else if (rovers[kID]->type == 2) {
			multiplek = 0.5;
		}
		else if (rovers[kID]->type == 3) {
			multiplek = 0.75;
		}
		else {
			multiplek = 1.0;
		}
		// END MODIFICATION

		gatheredVal = POIs[i]->val*(multiplej + multiplek);

		G += 2.0*(gatheredVal*Nij*Nik) / (dij + dik + 2.0);
	}
	return G;
}


matrix1d RoverDomain::getState(int me)
{
	// State elements/init
	matrix2d poisByQuadrantAndDistance(4, matrix1d(domain_params->distance_bounds.size(), 0.0)); // count POIs for each quadrant
	matrix2d roversByQuadrantAndDistance(4, matrix1d(domain_params->distance_bounds.size(), 0.0)); // count POIs for each quadrant
	matrix3d roverTypeCountByQuadrantAndDistance; // used if using types [QUADRANT][DIST][TYPE]
	matrix1d stateInputs;

	// Counting rovers by quadrant
	for (size_t i = 0; i<POIs.size(); i++) {
		int quadrant = int(easymath::cardinal_direction((XY)*rovers[me] - (XY)*POIs[i]));
		int dist = int(easymath::euclidean_distance((XY)*rovers[me], (XY)*POIs[i]));
		poisByQuadrantAndDistance[quadrant][dist] += 1 / POIs.size();	// Normalised count
	}

	for (size_t i = 0; i<roversByQuadrantAndDistance.size(); i++) {
		for (size_t j = 0; j<roversByQuadrantAndDistance[i].size(); j++) {
			roversByQuadrantAndDistance[i][j] /= rovers.size();
		}
	}

	// Stitch this together to form state...
	for (size_t i = 0; i<poisByQuadrantAndDistance.size(); i++) {
		stateInputs.insert(stateInputs.end(), poisByQuadrantAndDistance[i].begin(), poisByQuadrantAndDistance[i].end());
	}
	for (size_t i = 0; i<roversByQuadrantAndDistance.size(); i++) {
		stateInputs.insert(stateInputs.end(), roversByQuadrantAndDistance[i].begin(), roversByQuadrantAndDistance[i].end());
	}

	/* NOTE: REPLACE THIS WITH CHILD CLASS FOR TYPES

	if (usingTypes){
	// Collect and add type information if necessary
	roverTypeCountByQuadrantAndDistance = std::vector<std::vector<std::vector<double> > >(NSECTORS);
	for (int i=0; i<NSECTORS; i++){
	roverTypeCountByQuadrantAndDistance[i] = std::vector<std::vector<double> >(NDISTANCES);
	for (int j=0; j<NDISTANCES; j++){
	roverTypeCountByQuadrantAndDistance[i][j] = std::vector<double>(FAILTYPECOUNT);
	}
	}
	// Type count
	for (int i=0; i<rovers.size(); i++){
	std::pair<Direction,DistanceDivision> quadAndDist = relativePosition(rovers[me].x,rovers[me].y,rovers[i].x,rovers[i].y);
	int quadrant = int(quadAndDist.first);
	int dist = int(quadAndDist.second);
	roversByQuadrantAndDistance[quadrant][dist]++;
	roverTypeCountByQuadrantAndDistance[quadrant][dist][rovers[i].type]++;
	}
	// Normalization
	for (int i=0; i<roverTypeCountByQuadrantAndDistance.size(); i++){
	for (int j=0; j<roverTypeCountByQuadrantAndDistance[i].size(); j++){
	if (roversByQuadrantAndDistance[i][j]==0) continue;
	else {
	for (int k=0; k<roverTypeCountByQuadrantAndDistance[i][j].size(); k++){
	roverTypeCountByQuadrantAndDistance[i][j][k] /= roversByQuadrantAndDistance[i][j];
	}
	}
	}
	}

	// Stitching
	for (int i=0; i<roverTypeCountByQuadrantAndDistance.size(); i++){
	for (int j=0; j<roverTypeCountByQuadrantAndDistance[i].size(); j++){
	stateInputs.insert(stateInputs.end(), roverTypeCountByQuadrantAndDistance[i][j].begin(), roverTypeCountByQuadrantAndDistance[i][j].end());
	}
	}
	}*/

	// Generate state from stitched inputs
	return stateInputs;
}

/*

void RoverDomain::generatePOIs(){
// randomly generates POIs
POIs.clear(); // clears out previous POIs
for (int i=0; i<nPOIs; i++){
POIs.push_back(POI());
}
}
*/