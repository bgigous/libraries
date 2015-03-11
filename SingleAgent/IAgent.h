#pragma once
#include <vector>
class IAgent
{
public:
	IAgent(void);
	~IAgent(void);
	virtual std::vector<double> getAction(std::vector<double> state)=0; // gets an action given a state
	virtual void updatePolicyValues(double R)=0; // update current policy given a reward
};

