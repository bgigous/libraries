#include "RoverDomain.h"

using namespace std;
using namespace easymath;


RoverDomainParameters::RoverDomainParameters() :
	rover_move_scale(2.0),
	observation_distance(10.0),
	rover_types_fixed(vector<int>(n_rovers,Rover::SLOWTURN))
{
}

RoverDomainParameters::~RoverDomainParameters() {

}

Rover::Rover(GridWorld* world, RoverDomainParameters* domain_params_set) :
	XY(rand(0,world->xsize),rand(0,world->ysize)), domain_params(domain_params_set)
{

}

Rover::~Rover() {

}

POI::POI(XY p) : XY(p) {

}

POI::~POI() {

}

RoverDomain::RoverDomain(void) :
	domain_params(new RoverDomainParameters()),
	world(new GridWorld())
{
	// base class variables
	n_state_elements = domain_params->n_state_elements;
	n_control_elements = 2; // dx,dy
	n_steps = domain_params->n_steps;
	n_agents = domain_params->n_rovers;
	n_types = int(Rover::NUMTYPES);
	// end base class variables

	if (fixedRoverPositions.size()) {
		for (int i = 0; i < rovers.size(); i++) {
			(XY)*rovers[i] = fixedRoverPositions[i];
		}
		for (int i = 0; i < POIs.size(); i++) {
			*POIs[i] = fixedPOIPositions[i];
		}
	}
	else {
		printf("resetting positions");
		fixedRoverPositions = vector<XY>(n_agents);
		for (int i = 0; i < n_agents; i++) {
			rovers.push_back(new Rover(world,domain_params));
			fixedRoverPositions[i] = *rovers[i];
		}
		fixedPOIPositions = vector<XY>(POIs.size());
		for (int i = 0; i < POIs.size(); i++) {
			fixedPOIPositions[i] = *POIs[i];
		}
	}
}


RoverDomain::~RoverDomain(void)
{
	delete world;
	delete domain_params;

	for (int i = 0; i < rovers.size(); i++) {
		delete rovers[i];
	}

	for (int i = 0; i < POIs.size(); i++) {
		delete POIs[i];
	}
}

void RoverDomain::reset() {
	//resets the position of the rovers
	for (int i = 0; i < fixedRoverPositions.size(); i++) {
		(XY)*rovers[i] = fixedRoverPositions[i];
	}
}