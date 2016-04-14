#include "SectorGraphManager.h"

#include <vector>

using std::vector;
using easymath::XY;

SectorGraphManager::SectorGraphManager(matrix2d membership_map,
    vector<edge> edges) :
    membership_map(membership_map) {
    // Gets a grid map for each connection
    for (size_t i = 0; i < edges.size(); i++) {
        edge e = edges[i];
        m2graph[e.first][e.second]
            = new GridGraph(membership_map, e.first, e.second);
    }
}

SectorGraphManager::~SectorGraphManager(void) {
}

int SectorGraphManager::getMembership(const XY &p) {
    return static_cast<int>(membership_map[size_t(p.x)][size_t(p.y)]);
}

vector<XY> SectorGraphManager::astar(const XY &p1, const XY &p2) {
    int memstart = getMembership(p1);
    int memnext = getMembership(p2);
    return m2graph[memstart][memnext]->astar(p1, p2);
}
