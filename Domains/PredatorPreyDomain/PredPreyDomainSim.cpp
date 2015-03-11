#include "PredPreyDomainSim.h"

using namespace std;

PredPreyDomainSim::PredPreyDomainSim(void):
	domain_params(new PredatorPreyDomainParameters()),
	sim_params(new PredPreyDomainSimParameters()),
	NE_params(new NeuroEvoParameters(domain_params->numNonTypeElements,2))
{
	global_reward_log = matrix_2d(sim_params->n_runs); // initialize and keep static size
	for (int i=0; i<global_reward_log.size(); i++){
		global_reward_log[i] = matrix_1d(sim_params->n_epochs,0.0);
	}
}

/*void PredPreyDomainSim::runPredatorPrey(){
	// Runs two different simulations: the first with observable types, the second with no observable types

	//******** No types setting
	// Domain settings: uses default settings in PredatorPreyDomainParameters constructor
	domain_params->usingTypes = true;
	runSim();

	//******* Types setting
	domain_params->usingTypes = false;
	runSim();
}*/

/*void PredPreyDomainSim::runSim(){
	// Runs the simulation; no parameter modification

	for (int run=0; run<sim_params->n_runs; run++){
		clock_t tref = clock();
		printf("Run %i: ",run);
		simulatePredPreyRun(global_reward_log[run]);
		double timediff = double(clock()-tref);
		printf(" runtime = %f mins, time/epoch= %f seconds\n\n",timediff/(CLOCKS_PER_SEC*60.0), timediff/double(sim_params->n_epochs*CLOCKS_PER_SEC));
	}

	// Print out runGLogs to the appropriate file
	if (domain_params->usingTypes){
		PrintOut::toFile(global_reward_log,"predPreyTypes.csv");
	} else {
		PrintOut::toFile(global_reward_log,"predPreyNoTypes.csv");
	}
}*/

vector<vector<PredatorPreyDomain*> > PredPreyDomainSim::getIndependentDomains(){
	// Returns independent domains for one epoch (each domain run for only 1 episode)

	vector<vector<PredatorPreyDomain*> > independent_domains(NE_params->popSize*2); // hardcoded: 2k=100 neural networks... edit this later
	PredatorPreyDomain base_domain = PredatorPreyDomain(domain_params);  // 

	for (int i=0; i<independent_domains.size(); i++){
		independent_domains[i] = vector<PredatorPreyDomain*>(sim_params->n_trials);
		for (int j=0; j<independent_domains[i].size(); j++){
			independent_domains[i][j] = new PredatorPreyDomain(base_domain);
		}
	}

	return independent_domains;
}

vector<NeuroEvo*> PredPreyDomainSim::getNEForEachPredator(){
	vector<NeuroEvo*> NE_set(domain_params->nPredators);
	for (int i=0; i<domain_params->nPredators; i++){
		NE_set[i] = new NeuroEvo(NE_params);
	}
	return NE_set;
}

void PredPreyDomainSim::addExtraTypeInputs(vector<NeuroEvo*> &NESet){
	for (int i=0; i<NESet.size(); i++){
		for (list<NeuralNet*>::iterator j= NESet[i]->population.begin(); j!=NESet[i]->population.end(); j++){
			(*j)->addInputs(Predator::numTypes);
		}
	}
}

double PredPreyDomainSim::getAvgG(vector<vector<PredatorPreyDomain*> > &independent_domains){
	double avg = 1000.0; // scaling amount for reward
	
	for (vector<vector<PredatorPreyDomain*> >::iterator it=independent_domains.begin(); it!=independent_domains.end(); it++){
		int trialavg = 0;
		for (vector<PredatorPreyDomain*>::iterator dom=it->begin(); dom!=it->end(); dom++){
			for (vector<int>::iterator s=(*dom)->stepsToCapture.begin(); s!=(*dom)->stepsToCapture.end(); s++){
				trialavg += (*s);
			}
		}
		trialavg/=(*it->begin())->stepsToCapture.size(); // divide by number of prey (should be whole number)
		avg = (trialavg/double(it->size()))<avg? trialavg/double(it->size()):avg;
		
		// bug check
		if (avg==1){
			printf("Here's a bug?");
			for (vector<PredatorPreyDomain*>::iterator dom=it->begin(); dom!=it->end(); dom++){
				for (int p=0; p<(*dom)->prey.size(); p++){
					printf("(%i,%i)",(*dom)->prey[p]->x,(*dom)->prey[p]->y);
				}
				printf("\n");
			}
			system("pause");
		}
	}
	return avg;
}

/*void PredPreyDomainSim::simulatePredPreyRun(vector<double> &GLog){
	GLog = vector<double>(sim_params->n_epochs,0.0); // resets and allocates global reward logging

	vector<vector<PredatorPreyDomain*> > independent_domains = getIndependentDomains(); // generate domains to run in parallel
	vector<NeuroEvo*> NESet = getNEForEachPredator();

	if (domain_params->usingTypes) addExtraTypeInputs(NESet);

	for (int i=0; i<sim_params->n_epochs; i++){
		//printf("Epoch %i\n",i);
		if (i%50==0) printf("(%i/%i)",i,sim_params->n_epochs);
		else printf(".");
		
		simulatePredPreyEpoch(NESet,independent_domains);		
		
		//outputSteps();
		//GLog[i] = getAverageBestNNScore(NESet);
		GLog[i] = getAvgG(independent_domains);
	}

	// OBJECT CLEANUP
	for (int i=0; i<independent_domains.size(); i++){ // remove domains not being used
		for (int j=0; j<independent_domains[i].size(); j++){
			delete independent_domains[i][j];
		}
	}
	for (int i=0; i<NESet.size(); i++){
		delete NESet[i];
	}
}*/

/*
void PredPreyDomainSim::simulatePredPreyEpoch(vector<NeuroEvo*> &NESet, vector<vector<PredatorPreyDomain*> > &domains){
	// Domains: vector of domains that is population.size() x trials large, each an individual simulation
	// Prey movement: Random

	generateNewMembers(NESet); // NESet represents the predators; random prey movement
	// Preprocessing: access a set of neural networks given by each neuroEvo element
	int numNN = NESet[0]->population.size();
	vector<vector<NeuralNet*> > NNSets(numNN); // Set to the number of population members

	vector<list<NeuralNet*>::iterator> popMembersInNNSet(NESet.size()); // list of iterators to set members across NE objects
	for (int i=0; i<NESet.size(); i++){ // initialization of iterator list
		popMembersInNNSet[i]=NESet[i]->population.begin();
	}
	for (int i=0; i<numNN; i++){
		NNSets[i] = vector<NeuralNet*>(popMembersInNNSet.size());
		for (int j=0; j<popMembersInNNSet.size(); j++){
			NNSets[i][j] = *popMembersInNNSet[j];
			popMembersInNNSet[j]++; // iterate up when done with assignment
		}
	}

	for (int nthNN=0; nthNN<NESet[0]->population.size(); nthNN++){ // go through every neural network ...
		matrix_2d predFitnesses(domains[nthNN].size());
		matrix_2d preyFitnesses(domains[nthNN].size());

		for (int t=0; t<predFitnesses.size(); t++){
			//printf("t=%i, ",t);
			domains[nthNN][t]->simulatePredPreyEpisode(NNSets[nthNN],predFitnesses[t],preyFitnesses[t]); // pred/prey fitnesses set inside this
		}

		vector<double> predFit = mean2(predFitnesses);
		vector<double> preyFit = mean2(preyFitnesses);


		for (int i=0; i<NNSets[nthNN].size(); i++){
			NNSets[nthNN][i]->evaluation = predFit[i];
		}
	}

	for (int i=0; i<NESet.size(); i++){
		NESet[i]->selectSurvivors();
	}
}*/

void PredPreyDomainSim::generateNewMembers(vector<NeuroEvo*> &NESet){
	// create new members
	for (int i=0; i<NESet.size(); i++){
		NESet[i]->generateNewMembers();
	}
}

vector<double> PredPreyDomainSim::mean2(vector<vector<double> > myVector){
	vector<double> myMean(myVector[0].size(),0.0);

	for (int i=0; i<myVector.size(); i++){
		for (int j=0; j<myVector[i].size(); j++){
			myMean[j]+=myVector[i][j]/double(myVector.size());
		}
	}
	return myMean;
}

PredPreyDomainSim::~PredPreyDomainSim(void)
{
}