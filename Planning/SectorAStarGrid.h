#pragma once

#include "../../libraries/Planning/AStarGrid.h"
#include <map>
// A manager class to handle different instances of grid astar for different sectors

class SectorAStarGrid
{
public:
	SectorAStarGrid(Matrix<int,2> membership_map, vector<pair<int,int> > edges);
	~SectorAStarGrid(void);

	int getMembership(XY p);
	vector<XY> search(XY p1, XY p2);

private: 
	Matrix<int, 2> membership_map;
	std::map<int,std::map<int,AStarGrid*> > m2astar; // lets you know which A* to access
};