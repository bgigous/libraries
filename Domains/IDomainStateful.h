#pragma once

#include <vector>

class IDomainStateful
{
public:
	IDomainStateful(void);
	~IDomainStateful(void);

	virtual std::vector<std::vector<double> > getStates()=0; // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
	virtual std::vector<double> getRewards()=0; // Returns the reward vector for a set of agents [AGENTID]
	virtual std::vector<double> getPerformance()=0; // returns the performance vector for a set of agents
	virtual void simulateStep(std::vector<std::vector<double> > agent_actions)=0; // Simulates a step
	virtual void reset()=0;
	virtual void logStep(int step)=0;
	virtual void exportLog(std::string fid, double G)=0;

	// constants
	bool type_blind;
	int n_state_elements;
	int n_control_elements;
	int n_steps;
	int n_agents;
	int n_types;
	std::vector<int> fixed_types; // types fixed in the domain (MAYBE NOT HAVE THESE HERE?)

};

