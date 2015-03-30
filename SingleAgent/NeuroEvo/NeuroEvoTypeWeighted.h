#pragma once

#include "NeuroEvo.h"
#include "../NeuralNet/TypeNeuralNet.h"

class NeuroEvoTypeWeighted: public NeuroEvo
{
public:
	NeuroEvoTypeWeighted(NeuroEvoParameters* NE_params, int n_types, int n_state_elements):
		NeuroEvo(NE_params), n_types(n_types), n_state_elements(n_state_elements)
	{
		matrix3d preprocess_weights = zeros(n_types,n_state_elements,1); // last element not used; 1-1 relationship with state_element
		while (population.size()){
			delete population.front();
			population.pop_front();
		}
		

		// Neural network parameters
		// NOTE: this is after the "preprocess" step
		int INPUT = n_state_elements;
		int OUTPUT = params->nOutput;
		int HIDDEN = params->nHidden;
		for (int i=0; i<NE_params->popSize; i++){
			population.push_back(new TypeNeuralNet(INPUT, HIDDEN, OUTPUT, preprocess_weights));
		}
		params->nInput = INPUT;

		pop_member_active = population.begin();

	};

	virtual void generateNewMembers(){
		// Mutate existing members to generate more
		std::list<NeuralNet*>::iterator popMember=population.begin();
		for (int i=0; i<params->popSize; i++){ // add k new members
			//(*popMember)->evaluation = 0.0; // commented out so that you take parent's evaluation
			TypeNeuralNet* m = new TypeNeuralNet(*((TypeNeuralNet*)*popMember)); // dereference pointer AND iterator
			m->mutate();
			population.push_back(m);
			popMember++;
		}
	}

	int n_types, n_state_elements;

	using NeuroEvo::getAction; // so that the overloaded base class is seen
	
	matrix1d getAction(matrix2d state){
		// state has elements [type][state element]
		matrix1d preprocessed_state(n_state_elements,0.0);
		for (int s=0; s<n_state_elements; s++){
			for (int t=0; t<n_types; t++){
				preprocessed_state[s] += state[t][s]*((TypeNeuralNet*)(*pop_member_active))->preprocess_weights[t][s][0];
			}
		}
		return getAction(preprocessed_state);
	}
	
	~NeuroEvoTypeWeighted(){};
};