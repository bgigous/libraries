// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAV_H_
#define DOMAINS_UTM_UAV_H_


// STL includes
#include <queue>
#include <list>
#include <set>
#include <memory>
#include <utility>
#include <map>

// libraries includes
#include "../../Planning/TypeGraphManager.h"
#include "../../Planning/SectorGraphManager.h"
#include "UTMModesAndFiles.h"


class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */
 public:
    typedef std::pair<int, int> edge;

    UAV(int start_mem, int end_mem, UTMModes::UAVType t,
        TypeGraphManager* highGraph, std::map<edge, int>* linkIDs,
        UTMModes* params);

    ~UAV() {};


    void set_cur_link_ID(int link_ID) {
        cur_link_ID = link_ID;
    }

    UTMModes* params;

    int getDirection();  // gets the cardinal direction of the UAV

    virtual void planAbstractPath();

    std::list<int> getBestPath();  // does not set anything within the UAV

    int ID;
    size_t type_ID;
    UTMModes::UAVType type;

    double speed;  // connected to type_ID
    bool pathChanged;
    int mem, mem_end;
    std::list<int> high_path_prev;  // saves the high level path
    std::map<edge, int> *linkIDs;

    int next_link_ID;
    int cur_link_ID;

    //! Gets the sector ID from the location
    int nextSectorID(int n = 1);

    //! Gets the link ID from the location and desired next loction.
    //! Throws out of range if internal link.
    int curLinkID();

    //! Gets the link ID of the next 'hop'.
    //! Returns the current link if terminal
    int nextLinkID();

    //! Gets the sector ID from the location
    virtual int curSectorID();

    //! Gets the sector ID for the desired end location
    virtual int endSectorID();


    // Delay modeling/abstraction mode
    int t;

    // Reward calculation stuff
    std::set<int> sectors_touched;  // the sectors that the UAV has touched...
    std::set<int> links_touched;  // the sectors that the UAV has touched...

    bool currently_in_conflict;
    TypeGraphManager* highGraph;  // shared with the simulator (for now);
    bool on_internal_link;
};

class UAVDetail : public UAV {
 public:
    UAVDetail(easymath::XY start_loc, easymath::XY end_loc,
        UTMModes::UAVType t, TypeGraphManager* highGraph,
        std::map<edge, int>* linkIDs, UTMModes* params,
        SectorGraphManager* lowGraph);
    SectorGraphManager* lowGraph;


    // Physical location of a UAV
    easymath::XY loc;
    easymath::XY end_loc;
    std::queue<easymath::XY> target_waypoints;  // target waypoints, low-level
    void moveTowardNextWaypoint();  // takes a time increment to move over

    //! Gets the sector ID from the location
    virtual int curSectorID();
    //! Gets the sector ID for the desired end location
    virtual int endSectorID();

    void planDetailPath();
    virtual void planAbstractPath();
};
#endif  // DOMAINS_UTM_UAV_H_
