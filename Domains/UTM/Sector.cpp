// Copyright 2016 Carrie Rebhuhn
#include "Sector.h"
#include <vector>

Sector::Sector(easymath::XY xy, int sectorIDSet,
    int n_agents, std::vector<int> connections, UTMModes* params) :
    xy(xy),
    ID(sectorIDSet),
    connections(connections) {
}
