#include "SectorGraphManager.h"


SectorGraphManager::SectorGraphManager(Matrix<int,2> membership_map, vector<pair<int,int> > edges):
	membership_map(membership_map)
{
	// Gets a grid map for each connection
	for (unsigned int i=0; i<edges.size(); i++){
		pair<int,int> e = edges[i];
		m2graph[e.first][e.second] = new GridGraph(membership_map, e.first, e.second);
	}
}

SectorGraphManager::~SectorGraphManager(void)
{
}

int SectorGraphManager::getMembership(XY p){
	return membership_map(Numeric_lib::Index(p.x),Numeric_lib::Index(p.y));
}

vector<XY> SectorGraphManager::astar(XY p1, XY p2){
	int memstart = getMembership(p1);
	int memnext = getMembership(p2);
	return m2graph[memstart][memnext]->astar(p1,p2);
}