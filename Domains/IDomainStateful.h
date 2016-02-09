#pragma once

#include <vector>
#include <string>

typedef std::vector<double> matrix1d;
typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<std::vector<std::vector<double> > > matrix3d;

class IDomainStatefulParameters{
public:
	virtual int get_n_state_elements()=0;
	virtual int get_n_control_elements()=0;
	virtual int get_n_steps()=0;
	virtual int get_n_types()=0;
};

class IDomainStateful
{
public:
	IDomainStateful(IDomainStatefulParameters *params);
	~IDomainStateful(void);

	virtual matrix2d getStates()=0; // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
	virtual matrix3d getTypeStates()=0; // [AGENTID][TYPEID][STATEELEMENT]
	virtual matrix1d getRewards()=0; // Returns the reward vector for a set of agents [AGENTID]
	virtual matrix1d getPerformance()=0; // returns the performance vector for a set of agents
	virtual void simulateStep(matrix2d agent_actions)=0; // Simulates a step
	virtual void reset()=0;
	virtual void logStep()=0;
	virtual std::string createExperimentDirectory()=0; // creates a directory for the current domain's parameters
	int * step;
	virtual void synch_step(int* step)=0; // synchronizes the step with the rest of the sim


	// constants
	bool type_blind;
	int n_state_elements;
	int n_control_elements;
	int n_steps;
	int n_types;
	
	int n_agents;	// agents determined later!

};

