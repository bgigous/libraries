#include "World2D.h"


using namespace std;

World2D::World2D(void):
	world_params(new World2DParameters()),all_objects(new vector<Point2D*>())
{
}


World2D::~World2D(void)
{
	delete world_params;
	delete all_objects;
}
