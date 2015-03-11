#pragma once
#include "../Domains/IDomainStateful.h"
#include "../Multiagent/IMultiagentSystem.h"
#include "../FileIO/easyio/easyio.h"

/*
Instructions for using ISimulator:

1) declare child class of ISimulator
2) set Domain externally

*/

class ISimulator{
public:
	ISimulator(IDomainStateful* domain, IMultiagentSystem* MAS):
		domain(domain), MAS(MAS)
	{

	}
	~ISimulator(void){};
	IDomainStateful* domain;
	IMultiagentSystem* MAS;
	virtual void runExperiment(void)=0; // run the experiment
	void outputRewardLog(std::string reward_file){
		PrintOut::toFile1D(reward_log, reward_file);
	}
	void outputMetricLog(std::string metric_file){
		// Prints out log of performance metric/epoch
		PrintOut::toFile1D(metric_log,metric_file);
	}

	std::vector<double> reward_log;
	std::vector<double> metric_log;
};