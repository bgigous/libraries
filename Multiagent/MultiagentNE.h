#pragma once
#include "IMultiagentSystem.h"
#include "../SingleAgent/NeuroEvo/NeuroEvo.h"

class MultiagentNE :
	public IMultiagentSystem
{
public:
	MultiagentNE(void){};
	MultiagentNE(int n_agents, NeuroEvoParameters* NE_params):
		NE_params(NE_params)
	{
		for (int i=0; i<n_agents; i++){
			agents.push_back(new NeuroEvo(NE_params));
		}
	}
	~MultiagentNE(void){
		for (int i=0; i<agents.size(); i++){
			delete ((NeuroEvo*)agents[i]);
		}
	};
	void generateNewMembers(){
		// Generate new population members
		for (int i=0; i<agents.size(); i++){
			((NeuroEvo*)agents[i])->generateNewMembers();
		}
	}
	
	void selectSurvivors(){
		// Specific to Evo: select survivors
		for (int i=0; i<agents.size(); i++){
			((NeuroEvo*)agents[i])->selectSurvivors();
		}
	}
	
	bool setNextPopMembers(){
		// Kind of hacky; select the next member and return true if not at the end
		// Specific to Evo

		std::vector<bool> is_another_member(agents.size(),false);
		for (int i=0; i<agents.size(); i++){
			is_another_member[i] = ((NeuroEvo*)agents[i])->selectNewMember();
		}
		for (int i=0; i<is_another_member.size(); i++){
			if (!is_another_member[i]){
				return false;
			}
		}
		return true;
	}
	NeuroEvoParameters* NE_params;
};

