#include "NeuroEvo.h"

using namespace std;

NeuroEvoParameters::NeuroEvoParameters(int inputSet, int outputSet):
		nInput(inputSet), nOutput(outputSet), epsilon(0.1)
{
}


	void NeuroEvo::updatePolicyValues(double R){
		// Add together xi values, for averaging
		double xi=0.1; // "learning rate" for NE
		double V = (*pop_member_active)->evaluation;
		V = xi*(R-V)+V;
		(*pop_member_active)->evaluation = V;
	}

	std::vector<double> NeuroEvo::getAction(std::vector<double> state){
		return (*pop_member_active)->predictContinuous(state);
	}

NeuroEvo::NeuroEvo(NeuroEvoParameters* neuroEvoParamsSet)
{
	params = neuroEvoParamsSet;
	for (int i=0; i<params->popSize; i++){
		NeuralNet* nn = new NeuralNet(params->nInput,params->nHidden,params->nOutput);
		population.push_back(nn);
	}
	pop_member_active = population.begin();
}

void NeuroEvo::deletePopulation(){
	while (!population.empty()){
		delete population.back();
		population.pop_back();
	}
}

NeuroEvo::~NeuroEvo(void)
{
	deletePopulation();
}

bool NeuroEvo::selectNewMember(){
	pop_member_active++;
	if (pop_member_active==population.end()){
		pop_member_active = population.begin();
		return false;
	} else {
		return true;
	}
}

void NeuroEvo::generateNewMembers(){
	// Mutate existing members to generate more
	list<NeuralNet*>::iterator popMember=population.begin();
	for (int i=0; i<params->popSize; i++){ // add k new members
		//(*popMember)->evaluation = 0.0; // commented out so that you take parent's evaluation
		NeuralNet* m = new NeuralNet(**popMember); // dereference pointer AND iterator
		m->mutate();
		population.push_back(m);
		popMember++;
	}
}

double NeuroEvo::getBestMemberVal(){
	// Find the HIGHEST FITNESS value of any neural network
	double highest = population.front()->evaluation;
	for (list<NeuralNet*>::iterator popMember=population.begin(); popMember!=population.end(); popMember++){
		if (highest<(*popMember)->evaluation) highest=(*popMember)->evaluation;
	}
	return highest;
}

void listPointerShuffle(list<NeuralNet*> &L){
	vector<NeuralNet*> tmp(L.begin(),L.end());
	random_shuffle(tmp.begin(),tmp.end());
	copy(tmp.begin(),tmp.end(),L.begin());
}

void NeuroEvo::selectSurvivors(){
	// Select neural networks with the HIGHEST FITNESS
	population.sort(NNCompare); // Sort by the highest fitness
	int nExtraNN = population.size()-params->popSize;
	for (int i=0; i<nExtraNN; i++){ // Remove the extra
		delete population.back();
		population.pop_back();
	}
	//random_shuffle(population.begin(),population.end());
	listPointerShuffle(population);

	pop_member_active = population.begin();
}

void NeuroEvo::deepCopy(NeuroEvo &NE){
	// Creates new pointer addresses for the neural nets
	params = NE.params;
	
	deletePopulation();
	for (list<NeuralNet*>::iterator it=NE.population.begin(); it!=NE.population.end(); it++){
		population.push_back(new NeuralNet(**it));
	}
}