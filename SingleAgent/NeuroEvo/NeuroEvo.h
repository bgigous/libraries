#pragma once

#include "../NeuralNet/NeuralNet.h"
#include "../IAgent.h"
#include <set>
#include <utility>
#include <algorithm>
#include <list>


class NeuroEvoParameters{
public:
	NeuroEvoParameters(int inputSet, int outputSet);
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

	void deepCopy(NeuroEvo &NE);
	void deletePopulation(); // deletes all neural network population member pointers
	virtual void generateNewMembers(); // Generate k new members from existing population
	bool selectNewMember(); // Select the next member to test; if cannot be selected, end epoch
	double getBestMemberVal(); // get the highest evaluation in the group
	void setNNToBestMember();
	void selectSurvivors();
	static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {return (x->evaluation>y->evaluation);}

	void updatePolicyValues(double R);

	matrix1d getAction(matrix1d state);
	matrix1d getAction(matrix2d state);


	void save(std::string fileout){
		matrix2d nets;
		for (NeuralNet* p: population){
			matrix1d node_info;
			matrix1d wt_info;
			p->save(node_info,wt_info);
			nets.push_back(node_info);
			nets.push_back(wt_info);
		}

		PrintOut::toFile2D(nets,fileout);
	}

	void load(std::string filein){
		matrix2d netinfo;
		DataManip::load_variable(&netinfo,filein);

		for (int i=0; i<netinfo.size(); i+=2){
			NeuralNet nn = NeuralNet(1,1,1);
			nn.load(netinfo[i],netinfo[i+1]);
		}
	}
};

