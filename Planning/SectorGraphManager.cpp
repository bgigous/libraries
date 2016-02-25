#include "SectorGraphManager.h"

using namespace easymath;
using namespace std;

SectorGraphManager::SectorGraphManager(matrix2d membership_map, vector<Edge> edges):
	membership_map(membership_map)
{
	// Gets a grid map for each connection
	for (uint i=0; i<edges.size(); i++){
		pair<int,int> e = edges[i];
		m2graph[e.first][e.second] = new GridGraph(membership_map, e.first, e.second);
	}
}

SectorGraphManager::~SectorGraphManager(void)
{
}

int SectorGraphManager::getMembership(const XY &p){
	return (int)membership_map[uint(p.x)][uint(p.y)];
}

vector<XY> SectorGraphManager::astar(const XY &p1, const XY &p2){
	int memstart = getMembership(p1);
	int memnext = getMembership(p2);
	return m2graph[memstart][memnext]->astar(p1,p2);
}
