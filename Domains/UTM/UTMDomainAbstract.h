// Copyright Carrie Rebhuhn 2016
#ifndef DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define DOMAINS_UTM_UTMDOMAINABSTRACT_H_


// STL includes
#include <list>
#include <map>
#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../../Planning/TypeGraphManager.h"
#include "UAV.h"
#include "Sector.h"
#include "Link.h"
#include "Fix.h"
#include "../../FileIO/FileOut.h"

class UTMDomainAbstract :
    public IDomainStateful {
 public:
    typedef std::pair<int, int> edge;
    explicit UTMDomainAbstract(UTMModes* params);
    virtual ~UTMDomainAbstract(void);

    virtual void synch_step(int* step_set) {
        step = step_set;
        agents->steps = step_set;
        printf("set the step\n");
    }

    UTMModes* params;
    UTMFileNames* filehandler;


    // Agents
    IAgentManager* agents;

    // Moving parts
    std::vector<Sector*> sectors;
    std::vector<Link*> links;
    std::vector<Fix*> fixes;
    std::map<edge, int> *linkIDs;


    // Traffic
    std::list<UAV*> UAVs;
    void getNewUAVTraffic();  //! Adds traffic onto a link
    void absorbUAVTraffic();

    TypeGraphManager* highGraph;

    // Base function overloads
    matrix2d getStates();
    matrix3d getTypeStates();
    void simulateStep(matrix2d agent_actions);
    void logStep() {}  //! no logging currently
    std::string createExperimentDirectory();

    void exportSectorLocations(int fileID);

    // Different from children
    virtual matrix1d getPerformance();
    virtual matrix1d getRewards();
    virtual void incrementUAVPath();
    virtual void detectConflicts();

    virtual void getPathPlans();
    virtual void getPathPlans(const std::list<UAV*> new_UAVs);
    virtual void reset();


    //! Moves all it can in the list.
    //! Those eligible to move but who are blocked are left after the function.
    void try_to_move(std::vector<UAV*> * eligible_to_move);

    matrix2d last_cost_map;
};
#endif  // DOMAINS_UTM_UTMDOMAINABSTRACT_H_
