#include "GridWorld.h"

using namespace std;
using namespace easymath;

GridWorld::GridWorld(void):
	params(new GridParameters()),
	IDomainStateful(params),
	nRovers(20),
	deltaO(10.0),
	percentFail(0.80),
	xsize(100),
	ysize(100)
{
}

void GridWorld::generateStaticRoverPositions(){
	for (size_t i = 0; i < rovers.size(); i++)
	{
		staticRoverPositions[i] = XY(rand(0, xsize), rand(0, ysize));
		(XY)rovers[i] = staticRoverPositions[i];
	}
}

GridWorld::DistanceDivision GridWorld::distance_discretization(XY &p1, XY &p2){
	// Get the relative position of the second object to the first object
	double dist = easymath::manhattan_distance(p1, p2);
	if (dist<CLOSEBOUND) return CLOSE;
	else if (dist<MEDIUMBOUND) return MEDIUM;
	else return FAR;
}

void GridWorld::resetStaticRovers(){
	for (int i=0; i<staticRoverPositions.size(); i++){
		(XY)rovers[i] = staticRoverPositions[i];
	}
}

GridWorld::~GridWorld(void)
{
}


void GridWorld::roverRandomWalk(){
	for (int i=0; i<rovers.size(); i++){
		rovers[i].randWalk();
	}
}

void GridWorld::generateRovers(){ // 'fill' template function might be nice to have
	rovers.clear(); // clears out any Rovers that might have been lrking
	for (size_t i=0; i<nRovers; i++){
		rovers.push_back(GridRover(rand(0,xsize),rand(0,ysize)));
	}
}

void GridWorld::randomizePositions(std::vector<XY> &r) {
	set<XY> locs;
	while (locs.size() < r.size())
		locs.insert(XY(rand(0, xsize), rand(0, ysize)));
	for (XY &i : r) {
		i = *locs.begin();
		locs.erase(locs.begin());
	}
}