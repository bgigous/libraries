// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_IDOMAINSTATEFUL_H_
#define DOMAINS_IDOMAINSTATEFUL_H_

#include <vector>
#include <string>

typedef std::vector<double> matrix1d;
typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<std::vector<std::vector<double> > > matrix3d;

class IDomainStatefulParameters {
 public:
    virtual int get_n_state_elements() = 0;
    virtual int get_n_control_elements() = 0;
    virtual int get_n_steps() = 0;
    virtual int get_n_types() = 0;
};

class IDomainStateful {
 public:
    explicit IDomainStateful(IDomainStatefulParameters *params);
    ~IDomainStateful(void);
    // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
    virtual matrix2d getStates() = 0;
    // [AGENTID][TYPEID][STATEELEMENT]
    virtual matrix3d getTypeStates() = 0;
    // Returns the reward vector for a set of agents [AGENTID]

    virtual matrix1d getRewards() = 0;
    // returns the performance vector for a set of agents
    virtual matrix1d getPerformance() = 0;
    // Simulates a step
    virtual void simulateStep(matrix2d agent_actions) = 0;
    virtual void reset() = 0;
    virtual void logStep() = 0;
    virtual void exportStepsOfTeam(int team, std::string suffix) = 0;
    // creates a directory for the current domain's parameters
    virtual std::string createExperimentDirectory() = 0;
    int * step;
    // synchronizes the step with the rest of the sim
    virtual void synch_step(int* step) = 0;

    // constants
    bool type_blind;
    int n_state_elements;
    int n_control_elements;
    int n_steps;
    int n_types;

    int n_agents;   // agents determined later!
};

#endif  // DOMAINS_IDOMAINSTATEFUL_H_
