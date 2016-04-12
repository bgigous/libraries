// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_FIX_H_
#define DOMAINS_UTM_FIX_H_

#include "UAV.h"
#include <vector>
#include <list>
#include <utility>
#include <map>

class Fix;

class Fix {
 public:
    typedef std::pair<int, int> edge;
    Fix(easymath::XY loc, int ID, TypeGraphManager* highGraph,
        std::vector<Fix*>* fixes, UTMModes* params,
        std::map<edge, int> *linkIDs);


    ~Fix() {}
    UTMModes* params;
    std::list<UAV*> generateTraffic(int step);
    virtual bool atDestinationFix(const UAV &u);
    int ID;
    easymath::XY loc;
    std::map<edge, int>* linkIDs;
    virtual UAV* generate_UAV();

    TypeGraphManager* highGraph;
    std::vector<Fix*>* fixes;  // for generating destinations
};

class FixDetail : public Fix {
 public:
    FixDetail(easymath::XY loc, int ID, TypeGraphManager* highGraph,
        SectorGraphManager* lowGraph, std::vector<Fix*>* fixes,
        UTMModes* params, std::map<std::pair<int, int>, int> *linkIDs) :
        Fix(loc, ID, highGraph, fixes, params, linkIDs),
        lowGraph(lowGraph), approach_threshold(params->get_dist_thresh()),
        conflict_threshold(params->get_conflict_thresh())

    {};
    ~FixDetail() {}
    SectorGraphManager* lowGraph;
    virtual UAVDetail* generate_UAV();
    virtual bool atDestinationFix(const UAVDetail &u);


    // Approach
    double approach_threshold;
    double conflict_threshold;
};
#endif  // DOMAINS_UTM_FIX_H_

