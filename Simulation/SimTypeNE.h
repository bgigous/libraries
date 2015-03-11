#pragma once

#include "RoverDomain.h"
#include "TypeNeuroEvo.h"
#include "../../../master_libraries/easyio/easyio.h"
#include "MultiagentTypeNE.h"

class ISimTypeNEParameters{
public:
	ISimTypeNEParameters(){};
	~ISimTypeNEParameters(){};
	static const int n_epochs=200;
	static const int n_runs=1;
	static const int n_trials=1;
};

class SimTypeNE
{
public:
	SimTypeNE(IDomainStateful *domain);
	~SimTypeNE(void);

	MultiagentTypeNE* MAS;
	void experimentalRun(IDomainStateful* domain);
	void epoch(int ep, IDomainStateful *domain);
	bool type_blind;

	std::vector<double> reward_log;

private:
	ISimTypeNEParameters* sim_params;
	NeuroEvoParameters* NE_params;
	void generateStereotypes(IDomainStateful* domain);
	std::vector<std::vector<NeuroEvo*> > stereotypes;
};

