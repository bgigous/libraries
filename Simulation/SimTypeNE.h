#pragma once

#include "SimNE.h"
#include "../Multiagent/MultiagentTypeNE.h"

//#include "RoverDomain.h"
/*#include "TypeNeuroEvo.h"
#include "../../../master_libraries/easyio/easyio.h"*/

class ISimTypeNEParameters{
public:
	ISimTypeNEParameters(){};
	~ISimTypeNEParameters(){};
	static const int n_epochs=200;
	static const int n_runs=1;
	static const int n_trials=1;
};

class SimTypeNE: public SimNE
{
public:
	SimTypeNE(IDomainStateful *domain, MultiagentTypeNE::TypeHandling type_mode);
	SimTypeNE(IDomainStateful *domain, MultiagentNE* MAS, MultiagentTypeNE::TypeHandling type_mode);
	~SimTypeNE(void);
	ISimTypeNEParameters* sim_params;
	
	MultiagentTypeNE::TypeHandling type_mode;
	virtual matrix2d getActions(); // OVERLOADED VERSION OF SIMNE ... do we need to explicitly call this in function? ... yes
	//void runExperiment();
	//void epoch(int ep);

	// DEPRECATED AFTER THIS

	//void experimentalRun(IDomainStateful* domain);
	//void epoch(int ep, IDomainStateful *domain);
	//bool type_blind;

	//std::vector<double> reward_log;

//private:
	//ISimTypeNEParameters* sim_params;
	//NeuroEvoParameters* NE_params;
	//void generateStereotypes(IDomainStateful* domain);
	//std::vector<std::vector<NeuroEvo*> > stereotypes;
};

