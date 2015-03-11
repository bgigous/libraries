#include "PredPreyDomainSimTypeNE.h"

using namespace std;

PredPreyDomainSimTypeNE::PredPreyDomainSimTypeNE(void):
	sim_params(new PredPreyDomainSimParameters()),
	domainParams(new PredatorPreyDomainParameters()),
	NEParams(new NeuroEvoParameters(domainParams->numNonTypeElements,2))
{	
	// Create stereotype initializations
	generateStereoTypes();
}

void PredPreyDomainSimTypeNE::runPredatorPreyTypeNE(){
	// create a domain, with predators
	for (int i=0; i<sim_params->n_runs; i++){
		// DOMAIN SETUP, WITH TYPE GENERATION
		simulatePredPreyTypeNERun();
	}
}


void PredPreyDomainSimTypeNE::generateStereoTypes(){
	// Generate stereotype initializations using a practice domain
	// SIMPLE: return default NeuroEvo objects without training
	// TODO, COMPLEX: return trained NeuroEvo objects from a practice domain

	// stereotypes becomes a square matrix [my type][neighbor type], of neural networks trained in a practice domain
	stereotypes = vector<vector<NeuroEvo*> >(Predator::numTypes);
	for (int i=0; i<stereotypes.size(); i++){
		stereotypes[i] = vector<NeuroEvo*>(Predator::numTypes);
		for (int j=0; j<stereotypes[i].size(); j++){
			stereotypes[i][j] = new NeuroEvo(NEParams);
		}
	}
}


void PredPreyDomainSimTypeNE::simulatePredPreyTypeNERun(){
	// Get a domain
	PredatorPreyDomain* domain = new PredatorPreyDomain(domainParams);
	
	// Get a TypeNeuroEvo object for each predator
	vector<TypeNeuroEvo*> TNESet(domainParams->nPredators);
	for (int i=0; i<TNESet.size(); i++){
		TNESet[i] = new TypeNeuroEvo(NEParams,Predator::numTypes);
		TNESet[i]->deepCopyNETypes(stereotypes[domain->predators[i]->type]);
	}
	
	// Set up logging
	matrix_1d GLog(sim_params->n_epochs,0.0);

	// Simulate epochs
	for (int i=0; i<sim_params->n_epochs; i++){
		printf("Epoch %i\n",i);
		simulatePredPreyTypeNEEpoch(TNESet, domain, GLog[i]);
	}

	// Print out logging
	PrintOut::toFile(GLog,"TypeNEResults-1000.csv");
}

void PredPreyDomainSimTypeNE::simulatePredPreyTypeNEEpoch(vector<TypeNeuroEvo*> &TNESet, PredatorPreyDomain* domain, double &GLogElement){

	// Generate new members for each NeuroEvo object
	for (int i=0; i<TNESet.size(); i++){
		TNESet[i]->generateNewMembersAll();
	}

	// simulation/evaluation
	int k = NeuroEvoParameters::popSize;
	
	// preprocessing to get neural networks
	vector<vector<list<NeuralNet*>::iterator> > populationMembersInNNSet(TNESet.size()); // set of iterators to neural networks [predator][neighbortype]
	for (int i=0; i<TNESet.size(); i++){
		populationMembersInNNSet[i] = vector<list<NeuralNet*>::iterator>(TNESet[i]->NETypes.size());
		for (int j=0; j<TNESet[i]->NETypes.size(); j++){
			populationMembersInNNSet[i][j] = TNESet[i]->NETypes[j]->population.begin(); // [predator][neighbortype]
		}
	}
	vector<vector<vector<NeuralNet*> > > NNSet(k); // [nn][predator][neighbortype]
	for (int i=0; i<k; i++){
		NNSet[i] = vector<vector<NeuralNet*> >(populationMembersInNNSet.size());
		for (int pred=0; pred<NNSet[i].size(); pred++){
			NNSet[i][pred] = vector<NeuralNet*>(Predator::numTypes);
			for (int neighborType=0; neighborType<Predator::numTypes; neighborType++){
				NNSet[i][pred][neighborType] = *populationMembersInNNSet[pred][neighborType]; // assign neural network set
				populationMembersInNNSet[pred][neighborType]++; // increment iterator
			}
		}
	}

	for (int nthNN = 0; nthNN<k; nthNN++){
		matrix_2d predFitnesses(sim_params->n_trials);
		for (int t=0; t<sim_params->n_trials; t++){
			domain->simulatePredPreyEpisodeTypes(NNSet[nthNN],predFitnesses[t]);
		}
		matrix_1d predFit = easymath::mean2(predFitnesses);
		for (int i=0; i<NNSet[nthNN].size(); i++){
			for (int j=0; j<NNSet[nthNN][i].size(); j++){
				NNSet[nthNN][i][j]->evaluation = predFit[i];
			}
		}
	}

	GLogElement = 0.0;
	for (int i=0; i<TNESet.size(); i++){
		TNESet[i]->selectSurvivorsAll();
		vector<double> bestvals = TNESet[i]->getBestMemberValAll();
		for (int j=0; j<bestvals.size(); j++){
			GLogElement+= bestvals[j]/double(bestvals.size());
		}
	}
}

PredPreyDomainSimTypeNE::~PredPreyDomainSimTypeNE(void)
{
	delete domainParams;
	delete NEParams;
}
