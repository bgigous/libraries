#pragma once
#include "../GridWorld.h"
#include "../../SingleAgent/NeuroEvo/NeuroEvo.h"

#define SENSORFAILLIMIT 5.0 // amount that a 'sensorfail' type rover can deviate from desired course
enum FailType{
	NOMINAL, // no error
	SENSORFAIL, // takes inaccurate steps
	PROPULSIONLOSS, // stays in place
	PROPULSIONGAIN, // takes an extra step in whichever desired direction
	FAILTYPECOUNT // number of elements in enum
};


class RoverDomainParameters;

class Rover : public easymath::XY {
public:
	int ID;
	Rover(GridWorld* world, RoverDomainParameters* domain_params_set);
	RoverDomainParameters* domain_params;
	~Rover();
	GridWorld* world;
	const enum RoverType { FAST, SLOWTURN, ERRATIC, NORMAL, NUMTYPES };
	RoverType type;

	double get_type_multiplier() {
		if (type == FAST)
			return 0.25;
		else if (type == SLOWTURN)
			return 0.5;
		else
			return 0.75;
	}


	void walk(double dx, double dy);
	double get_x(void);
	void set_x(double new_x);
	double get_y(void);
	void set_y(double new_y);
};