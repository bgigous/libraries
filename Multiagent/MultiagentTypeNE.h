#pragma once
#include "MultiagentNE.h"
#include "TypeNeuroEvo.h"
#include <vector>

// Container for collection of 'Type Neuro Evo' agents
class MultiagentTypeNE: public MultiagentNE
{
public:
	MultiagentTypeNE(void){};

	MultiagentTypeNE(int n_agents, int n_types, NeuroEvoParameters* NE_params)
	{
		for (int i=0; i<n_agents; i++){
			agents.push_back(new TypeNeuroEvo(NE_params,n_types));
		}
	}
	~MultiagentTypeNE(void){
		for (int i=0; i<agents.size(); i++){
			delete agents[i];
		}
	}

	void initializeWithStereotypes(std::vector<std::vector<NeuroEvo*> > stereotypes, std::vector<int> agent_types){
		// Specific to Type NE: initialize all of the agents with its appropriate stereotype
		for (int i=0; i<agents.size(); i++){
			((TypeNeuroEvo*)agents[i])->deepCopyNETypes(stereotypes[agent_types[i]]);
		}
	}
	
};