#include "Rover.h"
using namespace easymath;

Rover::Rover(GridWorld* world_set, RoverDomainParameters* domain_params_set) :
	XY(rand(0, world_set->xsize), rand(0, world_set->ysize)),
	world(world_set), domain_params(domain_params_set)
{
	static int IDset = 0;
	ID = IDset;
	IDset++;
}


Rover::~Rover(void)
{
}

double Rover::get_x(void) {
	return x;
}

double Rover::get_y(void) {
	return y;
}

void Rover::set_y(double new_y) {
	if (new_y >= world->ysize)
		y = world->ysize - 1;
	else if (new_y < 0)
		y = 0;
	else
		y = new_y;
}

void Rover::set_x(double new_x) {
	if (new_x >= world->xsize)
		x = world->xsize - 1;
	else if (new_x < 0)
		x = 0;
	else
		x = new_x;
}

void Rover::walk(double dx, double dy){
	// Normal walking behavior
	set_x(x + dx);
	set_y(y + dy);
}