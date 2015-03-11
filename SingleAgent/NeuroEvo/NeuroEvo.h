#pragma once

#include "../NeuralNet/NeuralNet.h"
#include "../IAgent.h"
#include <set>
#include <utility>
#include <algorithm>
#include <list>


class NeuroEvoParameters{
public:
	NeuroEvoParameters(int inputSet, int outputSet):
		nInput(inputSet),
		nOutput(outputSet),
		epsilon(0.1)
	{}
	static const int nHidden = 50;
	static const int popSize=10; // surviving population size

	int nInput;
	int nOutput;
	double epsilon; // for epsilon-greedy selection: currently unused
};

class NeuroEvo: public IAgent 
{
public:
	NeuroEvo(){};
	NeuroEvo(NeuroEvoParameters* neuroEvoParamsSet);
	~NeuroEvo(void);
	
	// Class variables
	NeuroEvoParameters* params;
	std::list<NeuralNet*> population;
	std::list<NeuralNet*>::iterator pop_member_active;
	std::vector<double> getActiveMemberOutput(std::vector<double> inputs){
		// DEPRECATED--identical to getAction
		return (*pop_member_active)->predictContinuous(inputs);
	}

	void deepCopy(NeuroEvo &NE);
	void deletePopulation(); // deletes all neural network population member pointers
	void generateNewMembers(); // Generate k new members from existing population
	//std::vector<double> getOutput(std::vector<double> inputs); // during simulation, use current NN to get actions
	bool selectNewMember(); // Select the next member to test; if cannot be selected, end epoch
	double getBestMemberVal(); // get the highest evaluation in the group
	void setNNToBestMember();
	void selectSurvivors();
	static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {return (x->evaluation>y->evaluation);}

	void updatePolicyValues(double R){
		// Add together xi values, for averaging
		double xi=0.1; // "learning rate" for NE
		double V = (*pop_member_active)->evaluation;
		V = xi*(R-V)+V;
		(*pop_member_active)->evaluation = V;
	}

	std::vector<double> getAction(std::vector<double> state){
		return getActiveMemberOutput(state);
	}
};

