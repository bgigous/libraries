#pragma once

#include "NeuralNet.h"

class TypeNeuralNet: public NeuralNet
{
public:
	TypeNeuralNet(int input, int hidden, int output, matrix3d preprocess_weights):
		NeuralNet(input, hidden, output), preprocess_weights(preprocess_weights){
			for (int i=0; i<preprocess_weights.size(); i++){
				for (int j=0; j<preprocess_weights[i].size(); j++){
					for (int k=0; k<preprocess_weights[i][j].size(); k++){
						double fan_in = preprocess_weights.size();
						preprocess_weights[i][j][k] = randSetFanIn(fan_in);
					}
				}
			}
	}

	matrix3d preprocess_weights; // [t][s][t']

	void mutate(){
		NeuralNet::mutate();

		// now mutate the preprocess weights
		for (int i=0; i<preprocess_weights.size(); i++){
			for (int j=0; j<preprocess_weights[i].size(); j++){
				for (int k=0; k<preprocess_weights[i][j].size(); k++){
					double fan_in = double(preprocess_weights.size());
					preprocess_weights[i][j][k] += randAddFanIn(fan_in);
				}
			}
		}
	}

	~TypeNeuralNet(void);
};