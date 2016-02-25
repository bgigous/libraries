#include "SimNE.h"
#include "float.h"

/*
removed - causes memory leak
SimNE::SimNE(IDomainStateful* domain):
	ISimulator(domain,
	new MultiagentNE(domain->n_agents,
	new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements))),
	step(new int(0))
{
	domain->synch_step(step);
}
*/
// FOR DEBUGGING
SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS):
	ISimulator(domain, MAS),step(new int(0))
{
	domain->synch_step(step);
}

SimNE::~SimNE(void)
{
	delete step;
}

void SimNE::runExperiment(){
	for (int ep=0; ep<n_epochs; ep++){

		time_t epoch_start = time(NULL);
		this->epoch(ep);
		time_t epoch_end = time(NULL);
		time_t epoch_time = epoch_end - epoch_start;
		time_t run_time_left = (time_t(n_epochs-ep))*epoch_time;
		time_t run_end_time = epoch_end + run_time_left;

		char end_clock_time[26];
		#ifdef OS_WINDOWS
		ctime_s(end_clock_time, sizeof(end_clock_time), &run_end_time);
		#endif

		printf("Epoch %i took %i seconds.\n",ep,epoch_time);
		cout << "Estimated run end time: " << end_clock_time << endl;
	}
}

void SimNE::epoch(int ep){
	((MultiagentNE*)MAS)->generateNewMembers();
	double best_run = -DBL_MAX;
	double best_run_performance = -DBL_MAX;

	int n=0; // neural net number (for output file name)

	do{
		matrix2d Rtrials; // Trial average reward
		for (int t=0; t<n_trials; t++){
			clock_t tref = clock();
			for ((*step)=0; (*step)<domain->n_steps; (*step)++){
				matrix2d A = this->getActions(); // must be called by 'this' in order to access potential child class overload
				domain->simulateStep(A);
				domain->logStep();
			}
			//t= clock();
			//printf("t=%f\n",float(t-tref)/CLOCKS_PER_SEC);
			//tref=t;

			matrix1d R = domain->getRewards();
			matrix1d perf = domain->getPerformance();

			double avg_G = easymath::mean(R);
			double avg_perf = easymath::mean(perf);

			if (avg_G>best_run) {
				best_run = avg_G;
			}
			if (avg_perf>best_run_performance){
				best_run_performance = avg_perf;
			}

			printf("NN#%i, %f, %f, %f\n",n, best_run_performance, best_run, avg_perf);

			ostringstream epi,ni,ti;
			epi << ep;
			ni << n;
			ti << t;

			Rtrials.push_back(R);


			domain->reset();
		}
		MAS->updatePolicyValues(easymath::mean2(Rtrials)); // based on the trials...

		n++;
	} while (((MultiagentNE*)MAS)->setNextPopMembers()); // this also takes care of reset functions
	((MultiagentNE*)MAS)->selectSurvivors();

	reward_log.push_back(best_run);
	metric_log.push_back(best_run_performance);
}

matrix2d SimNE::getActions(){
	matrix2d S = domain->getStates();
	return MAS->getActions(S);
}
