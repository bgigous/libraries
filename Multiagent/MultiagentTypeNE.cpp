#include "MultiagentTypeNE.h"

MultiagentTypeNE::MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params, TypeHandling type_mode, int n_types):
	MultiagentNE(n_agents,NE_params), type_mode(type_mode),n_types(n_types)
{

}

MultiagentTypeNE::~MultiagentTypeNE(void){
	for (int i=0; i<agents.size(); i++){
		delete ((NeuroEvo*)agents[i]);
	}
}

matrix2d MultiagentTypeNE::getActions(matrix3d state){
	matrix2d actions(state.size()); // get an action set for each agent
	switch(type_mode){
	case MULTIMIND:
		{
			// vote among all TYPES for an action
			for (int i=0; i<agents.size(); i++){
				matrix1d action_i_sum = agents[i]->getAction(state[i][0]);
				for (int j=1; j<n_types; j++){ // starts at 1: initialized by 0
					matrix1d action_i_sum_temp = agents[i]->getAction(state[i][j]);
					for (int k=0; k<action_i_sum.size(); k++){
						action_i_sum[k] += action_i_sum_temp[k];
					}
				}
				for (int j=0; j<action_i_sum.size(); j++){
					action_i_sum[j] /= n_types; // normalize (magnitude unbounded for voting)
				}
				actions[i] = action_i_sum;
			}
			break;
		}
	default:
		{
			printf("Invalid type handling method. Please choose MULTIMIND.");
			system("pause");
			break;
		}
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