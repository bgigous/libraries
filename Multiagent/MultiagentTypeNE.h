#pragma once
#include "MultiagentNE.h"
#include "../SingleAgent/NeuroEvo/TypeNeuroEvo.h"
#include "../SingleAgent/NeuroEvo/NeuroEvo.h"
//#include "TypeNeuroEvo.h"
#include <vector>

// Container for collection of 'Type Neuro Evo' agents
class MultiagentTypeNE: public MultiagentNE
{
public:
	const enum TypeHandling{BLIND, WEIGHTED, CROSSWEIGHTED, MULTIMIND}; // options for handling different types
	TypeHandling type_mode;
	int n_types;

	MultiagentTypeNE(void){};

	MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params, TypeHandling type_mode, int n_types);
	~MultiagentTypeNE(void);

	//void initializeWithStereotypes(std::vector<std::vector<NeuroEvo*> > stereotypes, std::vector<int> agent_types);
	matrix2d getActions(matrix3d state);
};