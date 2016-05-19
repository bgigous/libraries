// Copyright 2016 Carrie Rebhuhn
#include "Sector.h"
#include <vector>

using easymath::XY;
using std::vector;

Sector::Sector(XY xy, int sectorIDSet, vector<int> connections,
    vector<XY> dest_locs, TypeGraphManager* highGraph,
    UTMModes* params, std::map<edge, int>* linkIDs) :
    xy(xy), ID(sectorIDSet), connections(connections)
	// Carrie! I made the generation_pt type Fix* instead of Fix
    //generation_pt(Fix(xy, sectorIDSet, highGraph, dest_locs, params, linkIDs)) 
{
}
