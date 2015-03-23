#pragma once
#include "IMultiagentSystem.h"
#include "../SingleAgent/NeuroEvo/NeuroEvo.h"

class MultiagentNE :
	public IMultiagentSystem
{
public:
	MultiagentNE(void);
	MultiagentNE(int n_agents, NeuroEvoParameters* NE_params);
	~MultiagentNE(void);
	void generateNewMembers();
	void selectSurvivors();
	bool setNextPopMembers();

	NeuroEvoParameters* NE_params;
};