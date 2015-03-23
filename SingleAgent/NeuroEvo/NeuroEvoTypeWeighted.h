#pragma once

#include "NeuroEvo.h"

class NeuroEvoTypeWeighted: public NeuroEvo
{
	NeuroEvoTypeWeighted(NeuroEvoParameters* NE_params, int n_types, int n_state_elements):
		n_types(n_types), n_state_elements(n_state_elements)
	{
	
	};
	int n_types, n_state_elements;
	matrix2d preprocess_weights; // [TYPE][STATE ELEMENT]
	
	matrix1d getPreprocessedState(matrix2d state){
		// state has elements [type][state element]

		matrix1d preprocessed_state(n_state_elements,0.0);
		for (int i=0; i<n_state_elements; i++){
			for (int j=0; j<n_types; j++){
				preprocessed_state[i] += state[i][j]*preprocess_weights[i][j];
			}
		}
		return preprocessed_state;
	}
	
	~NeuroEvoTypeWeighted(){};
};