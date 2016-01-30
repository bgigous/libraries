#pragma once

#include "../../libraries/Planning/GridGraph.h"
#include <map>
// A manager class to handle different instances of grid for different sectors

class SectorGraphManager
{
public:
	typedef pair<int,int> Edge;

	SectorGraphManager(matrix2d membership_map, vector<Edge> edges);
	~SectorGraphManager(void);

	int getMembership(easymath::XY p);
	vector<easymath::XY> astar(XY p1, XY p2);

private: 
	matrix2d membership_map;
	std::map<int,std::map<int,GridGraph*> > m2graph; // lets you know which A* to access
};