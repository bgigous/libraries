#pragma once
#include "../Math/easymath.h"

class IAgent
{
public:
	IAgent(void);
	virtual ~IAgent(void){};
	virtual matrix1d getAction(matrix1d state)=0; // gets an action given a state
	virtual matrix1d getAction(matrix2d state)=0; // gets an action given a 2d state
	virtual void updatePolicyValues(double R)=0; // update current policy given a reward
};

