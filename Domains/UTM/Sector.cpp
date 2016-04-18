// Copyright 2016 Carrie Rebhuhn
#include "Sector.h"
#include <vector>

Sector::Sector(easymath::XY xy, int sectorIDSet, std::vector<int> connections) :
    xy(xy),
    ID(sectorIDSet),
    connections(connections) {
}
