#include "SimTypeNE.h"

SimTypeNE::SimTypeNE(IDomainStateful* domain):
	sim_params(new ISimTypeNEParameters()),
	NE_params(new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements))
{
	generateStereotypes(domain);
	MAS = new MultiagentTypeNE(domain->n_agents,domain->n_types,NE_params);
	MAS->initializeWithStereotypes(stereotypes,domain->fixed_types);
}


SimTypeNE::~SimTypeNE(void)
{
	delete sim_params;
	delete NE_params;
	for (int i=0; i<stereotypes.size(); i++){
		for (int j=0; j<stereotypes.size(); j++){
			delete stereotypes[i][j];
		}
	}
	delete MAS;
}

void SimTypeNE::generateStereotypes(IDomainStateful* domain){
	stereotypes = vector<vector<NeuroEvo*> >(domain->n_types);
	for (int i=0; i<stereotypes.size(); i++){
		stereotypes[i] = vector<NeuroEvo*>(domain->n_types);
		for (int j=0; j<stereotypes[i].size(); j++){
			stereotypes[i][j] = new NeuroEvo(NE_params);
		}
	}
}

void SimTypeNE::experimentalRun(IDomainStateful *domain){ // This creates an entire experimental run
	for (int ep=0; ep<sim_params->n_epochs; ep++){
		printf("Epoch %i\n",ep);
		epoch(ep,domain);
	}

	if (type_blind){
		PrintOut::toFile1D(reward_log,"rewardLogBlind.csv");
	} else {
		PrintOut::toFile1D(reward_log,"rewardLogTypes.csv");
	}
}

void SimTypeNE::epoch(int ep, IDomainStateful *domain){
	MAS->generateNewMembers();
	domain->type_blind = type_blind;

	// Reward logging
	double best_run = -DBL_MAX; // initialize to 'really bad value'

	int n=0; // neural net number (for output file name)
	do{
		vector<vector<double> > Rtrials; // Trial average reward
		for (int t=0; t<sim_params->n_trials; t++){
			for (int s=0; s<domain->n_steps; s++){
				matrix2d S = domain->getStates();
				matrix2d A = MAS->getActions(S);
				domain->simulateStep(A);
				domain->logStep(s); // Logging function
			}

			matrix1d R = domain->getRewards();
			
			//Reward/path logging
			double avg_G = mean(R);
			if (avg_G>best_run) best_run = avg_G;
			printf("best_run=%f",best_run);
			ostringstream epi,ni,ti;
			epi << ep;
			ni << n;
			ti << t;

			// OVERRIDE!
			/*ti << 0;
			ni << 0;
			string fid = "epoch_"+epi.str()+"_n"+ni.str()+"_t"+ti.str()+".csv";
			domain->exportLog(fid,avg_G);*/
			
			Rtrials.push_back(R);
			domain->reset();
		}
		MAS->updatePolicyValues(mean2(Rtrials)); // based on the trials...

		n++;
	} while (MAS->setNextPopMembers()); // this also takes care of reset functions
	MAS->selectSurvivors();

	reward_log.push_back(best_run);
}