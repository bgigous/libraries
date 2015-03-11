#pragma once
#include "ISimulator.h"
#include "../Multiagent/MultiagentNE.h"
#include "../Math/easymath.h"
#include <limits>
#include <sstream>

using namespace std;
using namespace easymath;

class ISimNEParameters{
public:
	ISimNEParameters(void){};
	~ISimNEParameters(void){};
	static const int n_epochs = 100;
	static const int n_runs = 1;
	static const int n_trials = 1;
};

class SimNE: public ISimulator{
public:
	SimNE(IDomainStateful* domain):
		ISimulator(domain, new MultiagentNE(domain->n_agents, new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements))),
		sim_params(new ISimNEParameters())
	{};
	~SimNE(void){
		delete sim_params;
		delete ((MultiagentNE*)MAS)->NE_params;
		delete ((MultiagentNE*)MAS);
	};
	void runExperiment(){
		for (int ep=0; ep<sim_params->n_epochs; ep++){
			printf("Epoch %i\n",ep);
			epoch(ep);
		}
	}
	void epoch(int ep){
		((MultiagentNE*)MAS)->generateNewMembers();
		double best_run = -DBL_MAX;
		double best_run_performance = -DBL_MAX;

		int n=0; // neural net number (for output file name)
		
		do{
			matrix2d Rtrials; // Trial average reward
			for (int t=0; t<sim_params->n_trials; t++){
				clock_t tref = clock();
				for (int s=0; s<domain->n_steps; s++){
					//printf("Step %i\n",s);
					matrix2d S = domain->getStates();
					matrix2d A = MAS->getActions(S);
					domain->simulateStep(A);
					domain->logStep(s); // Logging function
				}
				t= clock();
				printf("t=%f\n",float(t-tref)/CLOCKS_PER_SEC);
				tref=t;

				matrix1d R = domain->getRewards();
				matrix1d perf = domain->getPerformance();

				//Reward/path logging
				double avg_G = mean(R);
				double avg_perf = mean(perf);
				if (avg_G>best_run) {
					best_run = avg_G;
					best_run_performance = avg_perf; // average of the performance metrics
					if (ep==0){
						domain->exportLog("stat_results/conflict_map-0-", ep); // blatant abuse of exportlog
					}
					if (ep==99){
						domain->exportLog("stat_results/conflict_map-99-",ep);
					}
				}
				printf("NN#%i, %f\n",n, best_run);
				ostringstream epi,ni,ti;
				epi << ep;
				ni << n;
				ti << t;
				Rtrials.push_back(R);
				domain->reset();
			}
			MAS->updatePolicyValues(mean2(Rtrials)); // based on the trials...

			n++;
		} while (((MultiagentNE*)MAS)->setNextPopMembers()); // this also takes care of reset functions
		((MultiagentNE*)MAS)->selectSurvivors();

		reward_log.push_back(best_run);
		metric_log.push_back(best_run_performance);
	}
	ISimNEParameters* sim_params;
};