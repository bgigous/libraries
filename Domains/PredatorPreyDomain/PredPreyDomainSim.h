#pragma once

#include "../../master_libraries/easyio/easyio.h"
#include <string>
#include "Classes\PredatorPreyDomain\PredatorPreyDomain.h"

using namespace std;

class PredPreyDomainSimParameters{
public:
	PredPreyDomainSimParameters(){};
	~PredPreyDomainSimParameters(){};
	static const int n_runs=1;
	static const int n_epochs=2;
	static const int n_trials=5;
	static const int n_steps=10;
};

class PredPreyDomainSim
{
public:
	PredPreyDomainSim(void);
	~PredPreyDomainSim(void);
	void runPredatorPrey();
	NeuroEvoParameters* NE_params;

private:
	//void runSim();
	std::vector<std::vector<PredatorPreyDomain*> > getIndependentDomains();
	std::vector<NeuroEvo*> getNEForEachPredator();
	double getAvgG(std::vector<std::vector<PredatorPreyDomain*> > &independent_domains);
	void addExtraTypeInputs(std::vector<NeuroEvo*> &NESet);
	//void simulatePredPreyRun(std::vector<double> &GLog);
	//void simulatePredPreyEpoch(std::vector<NeuroEvo*> &NESet, std::vector<std::vector<PredatorPreyDomain*> > &domains);
	void generateNewMembers(std::vector<NeuroEvo*> &NESet);
	matrix_1d mean2(matrix_2d myVector);

	PredatorPreyDomainParameters* domain_params;
	PredPreyDomainSimParameters* sim_params;
	matrix_2d global_reward_log; // Matrix of global rewards [run][epoch]
};