#pragma once

#include "../../libraries/Planning/GridGraph.h"
#include <map>
// A manager class to handle different instances of grid for different sectors

class SectorGraphManager
{
public:
	SectorGraphManager(Matrix<int,2> membership_map, vector<pair<int,int> > edges);
	~SectorGraphManager(void);

	int getMembership(XY p);
	vector<XY> astar(XY p1, XY p2);

private: 
	Matrix<int, 2> membership_map;
	std::map<int,std::map<int,GridGraph*> > m2graph; // lets you know which A* to access
};