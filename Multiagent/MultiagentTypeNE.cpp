#include "MultiagentTypeNE.h"

MultiagentTypeNE::MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params, TypeHandling type_mode, int n_types):
	MultiagentNE(n_agents,NE_params), type_mode(type_mode),n_types(n_types)
{
	// USING SWITCH STATEMENT FOR OBJECT CREATION. AFTER THIS POINT IN CODE, POLYMORPHISM USED.

	for (int i=0; i<agents.size(); i++){
		delete agents[i]; // delete the premade agents

		switch (type_mode){
		case MULTIMIND:
			{
				agents[i] = new TypeNeuroEvo(NE_params, n_types);
				break;
			}
		case WEIGHTED:
			{
				agents[i] = new NeuroEvoTypeWeighted(NE_params,n_types,NE_params->nInput); // each type plays a part simultaneously
				break;
			}
		case CROSSWEIGHTED:
			{
				agents[i] = new NeuroEvoTypeCrossweighted(NE_params, n_types,4); // each type plays a part simultaneously
				break;
			}
		}
	}
}

MultiagentTypeNE::~MultiagentTypeNE(void){
	/*for (int i=0; i<agents.size(); i++){
		delete ((NeuroEvo*)agents[i]);
	}*/
}

matrix2d MultiagentTypeNE::getActions(matrix3d state){
	matrix2d actions(state.size()); // get an action vector for each agent
	for (int i=0; i<agents.size(); i++){
		actions[i] = agents[i]->getAction(state[i]);
	}
	return actions;
}


/*
MultiagentTypeNE::MultiagentTypeNE(void){};

MultiagentTypeNE::MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params, TypeHandling type_mode, int n_types): 
MultiagentNE(n_agents, NE_params), type_mode(type_mode), n_types(n_types)
{
switch (type_mode){
case BLIND:
{
for (int i=0; i<n_agents; i++){
agents.push_back(NeuroEvo(NE_params));
}
break;
}
case WEIGHTED:
{
for (int i=0; i<n_agents; i++){
agents.push_back(NeuroEvoTypeWeighted(NE_params, n_types));
}
break;
}
case CROSSWEIGHTED:
{
for (int i=0; i<n_agents; i++){
agents.push_back(NeuroEvoTypeCrossweighted(NE_params, n_types, n_state_elements));
}
break;
}
case MULTIMIND:
{
for (int i=0; i<n_agents; i++){
agents.push_back(TypeNeuroEvo(NE_params, n_types));
}
}

}

for (int i=0; i<n_agents; i++){
agents.push_back(new TypeNeuroEvo(NE_params,n_types));
}
}

MultiagentTypeNE::~MultiagentTypeNE(void){
for (int i=0; i<agents.size(); i++){
delete agents[i];
}
}

void MultiagentTypeNE::initializeWithStereotypes(std::vector<std::vector<NeuroEvo*> > stereotypes, std::vector<int> agent_types){
// Specific to Type NE: initialize all of the agents with its appropriate stereotype
for (int i=0; i<agents.size(); i++){
((TypeNeuroEvo*)agents[i])->deepCopyNETypes(stereotypes[agent_types[i]]);
}
}*/